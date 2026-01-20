#include "evo_motor_driver.hpp"

EVO_Limit_Param evo_limit_param[EVO_Num_Of_Model] = {
    {12.5, 10.0, 50.0, 250.0, 50.0, 100.0},   // REVO_4310
    {12.5, 18.0, 150.0, 500.0, 5.0, 150.0},   // ENCOS_8108
};

EvoMotorDriver::EvoMotorDriver(uint16_t motor_id, std::string can_interface, uint16_t master_id_offset,
                               EVO_Motor_Model motor_model)
    : MotorDriver(), can_(SocketCAN::get(can_interface)), motor_model_(motor_model) {
    motor_id_ = motor_id;
    master_id_ = motor_id_ + master_id_offset;
    limit_param_ = evo_limit_param[motor_model_];
    CanCbkFunc can_callback = std::bind(&EvoMotorDriver::CanRxMsgCallback, this, std::placeholders::_1);
    can_->add_can_callback(can_callback, master_id_);
}

EvoMotorDriver::~EvoMotorDriver() { can_->remove_can_callback(master_id_); }

void EvoMotorDriver::MotorLock() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFC;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::MotorUnlock() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFD;
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

uint8_t EvoMotorDriver::MotorInit() {
    // send disable command to enter read mode
    EvoMotorDriver::MotorUnlock();
    Timer::ThreadSleepFor(normal_sleep_time);
    set_motor_control_mode(MIT);
    Timer::ThreadSleepFor(normal_sleep_time);
    // send enable command to enter contorl mode
    EvoMotorDriver::MotorLock();
    Timer::ThreadSleepFor(normal_sleep_time);
    EvoMotorDriver::refresh_motor_status();
    Timer::ThreadSleepFor(normal_sleep_time);
    switch (error_id_) {
        case EVOError::EVO_OVER_VOLTAGE:
            return EVOError::EVO_OVER_VOLTAGE;
            break;
        case EVOError::EVO_UNDER_VOLTAGE:
            return EVOError::EVO_UNDER_VOLTAGE;
            break;
        case EVOError::EVO_OVER_CURRENT:
            return EVOError::EVO_OVER_CURRENT;
            break;
        case EVOError::EVO_MOS_OVER_TEMP:
            return EVOError::EVO_MOS_OVER_TEMP;
            break;
        case EVOError::EVO_COIL_OVER_TEMP:
            return EVOError::EVO_COIL_OVER_TEMP;
            break;
        case EVOError::EVO_COMM_LOST:
            return EVOError::EVO_COMM_LOST;
            break;
        case EVOError::EVO_OVERLOAD:
            return EVOError::EVO_OVERLOAD;
            break;
        case EVOError::EVO_ENCODER_ERROR:
            return EVOError::EVO_ENCODER_ERROR;
            break;
        default:
            return error_id_;
    }
    return error_id_;
}

void EvoMotorDriver::MotorDeInit() {
    EvoMotorDriver::MotorUnlock();
    Timer::ThreadSleepFor(normal_sleep_time);
}

bool EvoMotorDriver::MotorWriteFlash() { return true; }

bool EvoMotorDriver::MotorSetZero() {
    // send set zero command
    EvoMotorDriver::EvoMotorSetZero();
    Timer::ThreadSleepFor(setup_sleep_time);  // wait for motor to set zero
    logger_->info("motor_id: {0}\tposition: {1}\t", motor_id_, get_motor_pos());
    EvoMotorDriver::MotorUnlock();
    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        logger_->warn("set zero error");
        return false;
    } else {
        logger_->info("set zero success");
        return true;
    }
    // disable motor
}

void EvoMotorDriver::CanRxMsgCallback(const can_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    uint16_t master_id_t = 0;
    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    
    master_id_t = rx_frame.can_id;
    
    if (motor_model_ == REVO_4310) {
        pos_int = rx_frame.data[1] << 8 | rx_frame.data[2];
        spd_int = rx_frame.data[3] << 4 | (rx_frame.data[4] & 0xF0) >> 4;
        t_int = (rx_frame.data[4] & 0x0F) << 8 | rx_frame.data[5];
        error_id_ = rx_frame.data[6];
        mos_temperature_ = rx_frame.data[7];
    } else {
        pos_int = rx_frame.data[1] << 8 | rx_frame.data[2];
        spd_int = rx_frame.data[3] << 4 | (rx_frame.data[4] & 0xF0) >> 4;
        t_int = (rx_frame.data[4] & 0x0F) << 8 | rx_frame.data[5];
        error_id_ = rx_frame.data[0] & 0x1F;
        mos_temperature_ = rx_frame.data[6];
        motor_temperature_ = rx_frame.data[7];
    }
    
    motor_pos_ = range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), 
                          -limit_param_.PosMax, limit_param_.PosMax);
    motor_spd_ = range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), 
                          -limit_param_.SpdMax, limit_param_.SpdMax);
    
    if (motor_model_ == REVO_4310) {
        motor_current_ = range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), 
                                  -limit_param_.TauMax, limit_param_.TauMax);
    } else {
        motor_current_ = range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), 
                                  -limit_param_.CUR_Max, limit_param_.CUR_Max);
    }
}

void EvoMotorDriver::MotorGetParam(uint8_t param_cmd) {
    can_frame tx_frame;
    tx_frame.can_id = 0x600 + motor_id_;
    tx_frame.can_dlc = 0x08;
    
    uint16_t index = 0x7000 + param_cmd;
    uint8_t subindex = 0x00;
    
    tx_frame.data[0] = 0x40;
    tx_frame.data[1] = index & 0xFF;
    tx_frame.data[2] = (index >> 8) & 0xFF;
    tx_frame.data[3] = subindex;
    tx_frame.data[4] = 0x00;
    tx_frame.data[5] = 0x00;
    tx_frame.data[6] = 0x00;
    tx_frame.data[7] = 0x00;
    
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::MotorPosModeCmd(float pos, float spd, bool ignore_limit) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    MotorMitModeCmd(pos, 0.0f, 100.0f, 5.0f, 0.0f);
}

void EvoMotorDriver::MotorSpdModeCmd(float spd) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    MotorMitModeCmd(0.0f, spd, 0.0f, 5.0f, 0.0f);
}

// Transmit MIT-mDme control(hybrid) package. Called in canTask.
void EvoMotorDriver::MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    uint16_t p, v, kp, kd, t;
    can_frame tx_frame;

    f_p = limit(f_p, -limit_param_.PosMax, limit_param_.PosMax);
    f_v = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(f_kp, KpMin, KpMax);
    f_kd = limit(f_kd, KdMin, getKdMax());
    f_t = limit(f_t, -limit_param_.TauMax, limit_param_.TauMax);

    int kd_bits = getKdBitWidth();
    
    p = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), bitmax<uint16_t>(16));
    v = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), bitmax<uint16_t>(12));
    kp = range_map(f_kp, KpMin, KpMax, uint16_t(0), bitmax<uint16_t>(12));
    kd = range_map(f_kd, KdMin, getKdMax(), uint16_t(0), bitmax<uint16_t>(kd_bits));
    t = range_map(f_t, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), bitmax<uint16_t>(12));

    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    if (motor_model_ == REVO_4310) {
        tx_frame.data[0] = p >> 8;
        tx_frame.data[1] = p & 0xFF;
        tx_frame.data[2] = v >> 4;
        tx_frame.data[3] = (v & 0x0F) << 4 | kp >> 8;
        tx_frame.data[4] = kp & 0xFF;
        tx_frame.data[5] = kd >> 4;
        tx_frame.data[6] = (kd & 0x0F) << 4 | t >> 8;
        tx_frame.data[7] = t & 0xFF;
    } else {
        uint8_t switch_user_mode = 0;
        tx_frame.data[0] = ((switch_user_mode & 0x07) << 5) | ((kp >> 7) & 0x1F);
        tx_frame.data[1] = ((kp & 0x7F) << 1) | ((kd >> 8) & 0x01);
        tx_frame.data[2] = kd & 0xFF;
        tx_frame.data[3] = (p >> 8) & 0xFF;
        tx_frame.data[4] = p & 0xFF;
        tx_frame.data[5] = (v >> 4) & 0xFF;
        tx_frame.data[6] = ((v & 0x0F) << 4) | ((t >> 8) & 0x0F);
        tx_frame.data[7] = t & 0xFF;
    }

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    motor_control_mode_ = MIT;
}

void EvoMotorDriver::EvoMotorSetZero() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFE;
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::EvoMotorClearError() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFD;
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::EvoWriteRegister(uint16_t index, uint8_t subindex, int32_t value) {
    can_frame tx_frame;
    tx_frame.can_id = 0x600 + motor_id_;
    tx_frame.can_dlc = 0x08;
    
    tx_frame.data[0] = 0x23;
    tx_frame.data[1] = index & 0xFF;
    tx_frame.data[2] = (index >> 8) & 0xFF;
    tx_frame.data[3] = subindex;
    tx_frame.data[4] = value & 0xFF;
    tx_frame.data[5] = (value >> 8) & 0xFF;
    tx_frame.data[6] = (value >> 16) & 0xFF;
    tx_frame.data[7] = (value >> 24) & 0xFF;
    
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::EvoReadRegister(uint16_t index, uint8_t subindex) {
    can_frame tx_frame;
    tx_frame.can_id = 0x600 + motor_id_;
    tx_frame.can_dlc = 0x08;
    
    tx_frame.data[0] = 0x40;
    tx_frame.data[1] = index & 0xFF;
    tx_frame.data[2] = (index >> 8) & 0xFF;
    tx_frame.data[3] = subindex;
    tx_frame.data[4] = 0x00;
    tx_frame.data[5] = 0x00;
    tx_frame.data[6] = 0x00;
    tx_frame.data[7] = 0x00;
    
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::EvoSaveRegister(uint8_t rid) {
    can_frame tx_frame;
    tx_frame.can_id = 0x600 + motor_id_;
    tx_frame.can_dlc = 0x08;
    
    tx_frame.data[0] = 0x2B;
    tx_frame.data[1] = 0x10;
    tx_frame.data[2] = 0x10;
    tx_frame.data[3] = rid;
    
    tx_frame.data[4] = 0x73;
    tx_frame.data[5] = 0x61;
    tx_frame.data[6] = 0x76;
    tx_frame.data[7] = 0x65;
    
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::refresh_motor_status() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFC;
    
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}
