// DmMotorDriver.cpp
#include "dm_motor_driver.hpp"

// extern std::shared_ptr<spdlog::logger> motor_logger;

DmMotorDriver::DmMotorDriver(uint16_t motor_id, std::string can_interface)
    : MotorDriver(), can_(SocketCAN::get(can_interface)) {
    motor_id_ = motor_id;
    CanCbkCondition can_condition = std::bind(
        [motor_id](const can_frame& frame) {
            // std::cout << (uint16_t)(frame.data[0] & 0x0F) << std::endl;
            return (bool)((uint16_t)((frame.data[0] & 0x0F) == motor_id));
        },
        std::placeholders::_1);
    CanCbkFunc can_callback = std::bind(&DmMotorDriver::CanRxMsgCallback, this, std::placeholders::_1);
    can_->add_can_callback(
        std::make_tuple(std::string("motor#") + std::to_string(motor_id_), can_condition, can_callback));
}

DmMotorDriver::~DmMotorDriver() {
    can_->remove_can_callback(std::string("motor#") + std::to_string(motor_id_));
}

void DmMotorDriver::MotorLock() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;  // change according to the mode
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
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::MotorUnlock() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;  // change according to the mode
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
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

uint8_t DmMotorDriver::MotorInit() {
    // send disable command to enter read mode
    DmMotorDriver::MotorUnlock();
    Timer::ThreadSleepFor(normal_sleep_time);
    // send enable command to enter contorl mode
    DmMotorDriver::MotorLock();
    Timer::ThreadSleepFor(normal_sleep_time);
    switch (error_id_) {
        case DMError::DM_DOWN:
            return DMError::DM_DOWN;
            break;
        case DMError::DM_UP:
            return DMError::DM_UP;
            break;
        case DMError::LOST_CONN:
            return DMError::LOST_CONN;
            break;
        case DMError::OVER_CURRENT:
            return DMError::OVER_CURRENT;
            break;
        case DMError::MOS_OVER_TEMP:
            return DMError::MOS_OVER_TEMP;
            break;
        case DMError::COIL_OVER_TEMP:
            return DMError::COIL_OVER_TEMP;
            break;
        case DMError::UNDER_VOLT:
            return DMError::UNDER_VOLT;
            break;
        case DMError::OVER_VOLT:
            return DMError::OVER_VOLT;
            break;
        case DMError::OVER_LOAD:
            return DMError::OVER_LOAD;
            break;
        default:
            return error_id_;
    }
    return error_id_;
}

void DmMotorDriver::MotorDeInit() {
    DmMotorDriver::MotorUnlock();
    Timer::ThreadSleepFor(normal_sleep_time);
}

bool DmMotorDriver::MotorWriteFlash() { return true; }

bool DmMotorDriver::MotorSetZero() {
    // send set zero command
    DmMotorDriver::DmMotorSetZero();
    Timer::ThreadSleepFor(500);  // wait for motor to set zero
    // motor_logger->info("motor_id: %d\tposition: %f\t", motor_id_,
    //                    get_motor_pos());
    logger_->info("motor_id: {0}\tposition: {1}\t", motor_id_, get_motor_pos());
    DmMotorDriver::MotorUnlock();
    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        logger_->warn("set zero error");
        return false;
    } else {
        logger_->info("set zero success");
        return false;
    }
    // disable motor
}

void DmMotorDriver::CanRxMsgCallback(const can_frame& rx_frame) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count--;
    }
    if (((uint16_t)(rx_frame.data[0] & 0x0F) == motor_id_)) {
        uint16_t motor_id_t = 0;
        uint16_t pos_int = 0;
        uint16_t spd_int = 0;
        uint16_t t_int = 0;
        pos_int = rx_frame.data[1] << 8 | rx_frame.data[2];
        spd_int = rx_frame.data[3] << 4 | (rx_frame.data[4] & 0xF0) >> 4;
        t_int = (rx_frame.data[4] & 0x0F) << 8 | rx_frame.data[5];
        motor_id_t = (rx_frame.data[0] & 0x0F);
        if ((rx_frame.data[0] & 0xF0) >> 4 > 7) {  // error code range from 8 to 15
            error_id_ = (rx_frame.data[0] & 0xF0) >> 4;
        }
        motor_pos_ = range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), kPMin, kPMax);
        motor_spd_ = range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), kSpdMin, kSpdMax);
        motor_current_ = range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), kTorqueMin, kTorqueMax);
        mos_temperature_ = rx_frame.data[6];
        motor_temperature_ = rx_frame.data[7];

        heartbeat_detect_counter_ = 0;
    }
}

void DmMotorDriver::MotorGetParam(uint8_t param_cmd) {
    can_frame tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0x33;
    tx_frame.data[3] = param_cmd;

    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFF;
    can_->transmit(tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::MotorPosModeCmd(float pos, float spd, bool ignore_limit) {
    if (motor_control_mode_ != POS) {
        set_motor_control_mode(POS);
        return;
    }
    if (pos < joint_lower_bounder_[motor_id_ - 1] || pos > joint_upper_bounder_[motor_id_ - 1]) {
        logger_->warn("motor {0} pos {1} is out of jointspace: {2} {3}", motor_id_, pos,
                      joint_lower_bounder_[motor_id_ - 1], joint_upper_bounder_[motor_id_ - 1]);
        return;
    }
    can_frame tx_frame;
    tx_frame.can_id = 0x100 + motor_id_;
    tx_frame.can_dlc = 0x08;
    uint8_t *pbuf, *vbuf;

    spd = limit(spd, kSpdMin, kSpdMax);
    pos = limit(pos, kPMin, kPMax);

    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&spd;

    tx_frame.data[0] = *pbuf;
    tx_frame.data[1] = *(pbuf + 1);
    tx_frame.data[2] = *(pbuf + 2);
    tx_frame.data[3] = *(pbuf + 3);
    tx_frame.data[4] = *vbuf;
    tx_frame.data[5] = *(vbuf + 1);
    tx_frame.data[6] = *(vbuf + 2);
    tx_frame.data[7] = *(vbuf + 3);

    can_->transmit(tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::MotorSpdModeCmd(float spd) {
    if (motor_control_mode_ != SPD) {
        set_motor_control_mode(SPD);
        return;
    }
    can_frame tx_frame;
    tx_frame.can_id = 0x200 + motor_id_;
    tx_frame.can_dlc = 0x04;

    union32_t rv_type_convert;
    rv_type_convert.f = spd;
    tx_frame.data[0] = rv_type_convert.buf[0];
    tx_frame.data[1] = rv_type_convert.buf[1];
    tx_frame.data[2] = rv_type_convert.buf[2];
    tx_frame.data[3] = rv_type_convert.buf[3];

    can_->transmit(tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

// Transmit MIT-mDme control(hybrid) package. Called in canTask.
void DmMotorDriver::MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    uint16_t p, v, kp, kd, t;
    can_frame tx_frame;

    f_p = limit(f_p, kPMin, kPMax);
    f_v = limit(f_v, kSpdMin, kSpdMax);
    f_kp = limit(f_kp, kKpMin, kKpMax);
    f_kd = limit(f_kd, kKdMin, kKdMax);
    f_t = limit(f_t, kTorqueMin, kTorqueMax);

    p = range_map(f_p, kPMin, kPMax, uint16_t(0), bitmax<uint16_t>(16));
    v = range_map(f_v, kSpdMin, kSpdMax, uint16_t(0), bitmax<uint16_t>(12));
    kp = range_map(f_kp, kKpMin, kKpMax, uint16_t(0), bitmax<uint16_t>(12));
    kd = range_map(f_kd, kKdMin, kKdMax, uint16_t(0), bitmax<uint16_t>(12));
    t = range_map(f_t, kTorqueMin, kTorqueMax, uint16_t(0), bitmax<uint16_t>(12));

    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = p >> 8;
    tx_frame.data[1] = p & 0xFF;
    tx_frame.data[2] = v >> 4;
    tx_frame.data[3] = (v & 0x0F) << 4 | kp >> 8;
    tx_frame.data[4] = kp & 0xFF;
    tx_frame.data[5] = kd >> 4;
    tx_frame.data[6] = (kd & 0x0F) << 4 | t >> 8;
    tx_frame.data[7] = t & 0xFF;

    can_->transmit(tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

// todo
void DmMotorDriver::MotorSetPosParam(float kp, float kd) {}

void DmMotorDriver::MotorSetSpdParam(float kp, float ki) {}

void DmMotorDriver::MotorSetFilterParam(float position_kd_filter, float kd_spd) {}

void DmMotorDriver::set_motor_id(uint8_t motor_id) {}

void DmMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    DmWriteRegister(10, motor_control_mode);
    motor_control_mode_ = motor_control_mode;
}

void DmMotorDriver::DmMotorSetZero() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;  // change according to the mode
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
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::DmMotorClearError() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;  // change according to the mode
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFB;
    can_->transmit(tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::DmWriteRegister(uint8_t rid, float value) {
    param_cmd_flag_[rid] = false;
    can_frame tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x08;

    uint8_t* vbuf;
    vbuf = (uint8_t*)&value;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0x55;
    tx_frame.data[3] = rid;

    tx_frame.data[4] = *vbuf;
    tx_frame.data[5] = *(vbuf + 1);
    tx_frame.data[6] = *(vbuf + 2);
    tx_frame.data[7] = *(vbuf + 3);
    can_->transmit(tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::DmWriteRegister(uint8_t rid, int32_t value) {
    param_cmd_flag_[rid] = false;
    can_frame tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x08;

    uint8_t* vbuf;
    vbuf = (uint8_t*)&value;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0x55;
    tx_frame.data[3] = rid;

    tx_frame.data[4] = *vbuf;
    tx_frame.data[5] = *(vbuf + 1);
    tx_frame.data[6] = *(vbuf + 2);
    tx_frame.data[7] = *(vbuf + 3);
    can_->transmit(tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::DmSaveRegister(uint8_t rid) {
    can_frame tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0xAA;
    tx_frame.data[3] = rid;

    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFF;
    can_->transmit(tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::refresh_motor_status() {
    can_frame tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0xCC;
    tx_frame.data[3] = 0x00;

    tx_frame.data[4] = 0x00;
    tx_frame.data[5] = 0x00;
    tx_frame.data[6] = 0x00;
    tx_frame.data[7] = 0x00;
    can_->transmit(tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}