#include "evo_motor_driver.hpp"

EVO_Limit_Param evo_limit_param[EVO_Num_Of_Motor] = {
    {12.5, 20.0, 18.0, 500.0, 5.0},     // EVO431040
    {12.5, 10.0, 50.0, 250.0, 50.0},    // EVO811825
    {12.5, 10.0, 50.0, 250.0, 50.0},    // EVO811832
};

EvoMotorDriver::EvoMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                               EVO_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model) {
    if (interface_type != "can" && interface_type != "canfd" && interface_type != "ethercanfd") {
        throw std::runtime_error("EVO driver only supports CAN and CAN-FD interfaces");
    }
    motor_id_ = motor_id;
    limit_param_ = evo_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;

    if (interface_type == "canfd" || interface_type == "ethercanfd") {
        comm_type_ = CommType::CANFD;
        motor_index_ = (motor_id_ > 0 && motor_id_ <= 8) ? (motor_id_ - 1) : 0;
        canfd_ = MotorsCANFD::get(can_interface);

        CanFdCbkFunc canfd_callback = std::bind(&EvoMotorDriver::canfd_rx_cbk, this, std::placeholders::_1);
        canfd_->add_canfd_callback(canfd_callback, motor_id_);
        std::lock_guard<std::mutex> lock(bus_registry_mutex_);
        bus_registry_[can_interface_].push_back(this);
    } else if (interface_type == "can") {
        comm_type_ = CommType::CAN;
        can_ = MotorsCAN::get(can_interface);
        
        CanCbkFunc can_callback = std::bind(&EvoMotorDriver::can_rx_cbk, this, std::placeholders::_1);
        can_->add_can_callback(can_callback, motor_id_);
    } 
}

EvoMotorDriver::~EvoMotorDriver() { 
    if (comm_type_ == CommType::CANFD) {
        canfd_->remove_canfd_callback(motor_id_);
        std::lock_guard<std::mutex> lock(bus_registry_mutex_);
        auto& motors = bus_registry_[can_interface_];
        motors.erase(std::remove(motors.begin(), motors.end(), this), motors.end());
        if (motors.empty()) {
            bus_registry_.erase(can_interface_);
        }
    } else if (comm_type_ == CommType::CAN) {
        can_->remove_can_callback(motor_id_);
    }
}

void EvoMotorDriver::lock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = EVO_CMD_ENABLE;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
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
        tx_frame.data[7] = EVO_CMD_ENABLE;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::unlock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;
        
        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = EVO_CMD_DISABLE;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
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
        tx_frame.data[7] = EVO_CMD_DISABLE;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

uint8_t EvoMotorDriver::init_motor() {
    // send disable command to enter read mode
    EvoMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
    set_motor_control_mode(MIT);
    Timer::sleep_for(normal_sleep_time);
    // send enable command to enter contorl mode
    EvoMotorDriver::lock_motor();
    Timer::sleep_for(normal_sleep_time);
    EvoMotorDriver::refresh_motor_status();
    Timer::sleep_for(normal_sleep_time);
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

void EvoMotorDriver::deinit_motor() {
    EvoMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool EvoMotorDriver::write_motor_flash() { 
    return true;
}

bool EvoMotorDriver::set_motor_zero() {
    // send set zero command
    EvoMotorDriver::set_motor_zero_evo();
    Timer::sleep_for(setup_sleep_time);
    EvoMotorDriver::refresh_motor_status();
    Timer::sleep_for(setup_sleep_time);  // wait for motor to set zero
    logger_->info("motor_id: {0}\tposition: {1}\t", motor_id_, get_motor_pos());
    EvoMotorDriver::unlock_motor();
    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        logger_->warn("set zero error");
        return false;
    } else {
        logger_->info("set zero success");
        return true;
    }
    // disable motor
}

void EvoMotorDriver::can_rx_cbk(const can_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    pos_int = rx_frame.data[1] << 8 | rx_frame.data[2];
    spd_int = rx_frame.data[3] << 4 | (rx_frame.data[4] & 0xF0) >> 4;
    t_int = (rx_frame.data[4] & 0x0F) << 8 | rx_frame.data[5];
    error_id_ = rx_frame.data[6];
    if (error_id_ > 0) {
            if (logger_) {
            logger_->error("can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}", can_interface_, motor_id_, (uint32_t)error_id_);
        }
    }
    motor_pos_ = 
        range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.PosMax, limit_param_.PosMax) + motor_zero_offset_;
    motor_spd_ = 
        range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.SpdMax, limit_param_.SpdMax);
    motor_current_ = 
        range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.TauMax, limit_param_.TauMax);
    mos_temperature_ = rx_frame.data[7];
    motor_temperature_ = rx_frame.data[7];
}

void EvoMotorDriver::canfd_rx_cbk(const canfd_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    if (rx_frame.len < 8) return;

    if (!(rx_frame.flags & CANFD_FDF)) {
        can_frame frame{};
        frame.can_id = rx_frame.can_id;
        frame.can_dlc = rx_frame.len;
        memcpy(frame.data, rx_frame.data, 8);
        can_rx_cbk(frame);
        return;
    }

    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    pos_int = rx_frame.data[0] << 8 | rx_frame.data[1];
    spd_int = rx_frame.data[2] << 4 | (rx_frame.data[3] >> 4);
    t_int = ((rx_frame.data[3] & 0x0F) << 8) | rx_frame.data[4];
    error_id_ = ((rx_frame.data[6] << 8) | rx_frame.data[7]) >> 1 & 0x7F;
    if (error_id_ > 0) {
        if (logger_) {
            logger_->error("can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}", can_interface_, motor_id_, (uint32_t)error_id_);
        }
    }
    motor_pos_ = 
        range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.PosMax, limit_param_.PosMax) + motor_zero_offset_;
    motor_spd_ = 
        range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.SpdMax, limit_param_.SpdMax);
    motor_current_ = 
        range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.TauMax, limit_param_.TauMax);
    mos_temperature_ = static_cast<float>(rx_frame.data[5]) - 40.0f;
    motor_temperature_ = static_cast<float>(rx_frame.data[5]) - 40.0f;
}

void EvoMotorDriver::get_motor_param(uint8_t param_cmd) {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = EVO_CMD_START_FLASH;
        tx_frame.data[1] = param_cmd;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = 0x00;
        tx_frame.data[5] = 0x00;
        tx_frame.data[6] = EVO_CMD_READ_FLASH;
        tx_frame.data[7] = EVO_CMD_END_FLASH;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.can_dlc = 0x08;
        
        tx_frame.data[0] = EVO_CMD_START_FLASH;
        tx_frame.data[1] = param_cmd;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = 0x00;
        tx_frame.data[5] = 0x00;
        tx_frame.data[6] = EVO_CMD_READ_FLASH;
        tx_frame.data[7] = EVO_CMD_END_FLASH;
        
        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
    if (motor_control_mode_ != POS) {
        set_motor_control_mode(POS);
        return;
    }
    uint8_t *pbuf, *vbuf;

    pos -= motor_zero_offset_;
    spd = limit(spd, -limit_param_.SpdMax, limit_param_.SpdMax);
    pos = limit(pos, -limit_param_.PosMax, limit_param_.PosMax);

    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&spd;

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = *pbuf;
        tx_frame.data[1] = *(pbuf + 1);
        tx_frame.data[2] = *(pbuf + 2);
        tx_frame.data[3] = *(pbuf + 3);
        tx_frame.data[4] = *vbuf;
        tx_frame.data[5] = *(vbuf + 1);
        tx_frame.data[6] = *(vbuf + 2);
        tx_frame.data[7] = *(vbuf + 3);

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = *pbuf;
        tx_frame.data[1] = *(pbuf + 1);
        tx_frame.data[2] = *(pbuf + 2);
        tx_frame.data[3] = *(pbuf + 3);
        tx_frame.data[4] = *vbuf;
        tx_frame.data[5] = *(vbuf + 1);
        tx_frame.data[6] = *(vbuf + 2);
        tx_frame.data[7] = *(vbuf + 3);

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::motor_spd_cmd(float spd) {
    if (motor_control_mode_ != SPD) {
        set_motor_control_mode(SPD);
        return;
    }
    spd = limit(spd, -limit_param_.SpdMax, limit_param_.SpdMax);
    union32_t rv_type_convert;
    rv_type_convert.f = spd;

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = rv_type_convert.buf[0];
        tx_frame.data[1] = rv_type_convert.buf[1];
        tx_frame.data[2] = rv_type_convert.buf[2];
        tx_frame.data[3] = rv_type_convert.buf[3];

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = rv_type_convert.buf[0];
        tx_frame.data[1] = rv_type_convert.buf[1];
        tx_frame.data[2] = rv_type_convert.buf[2];
        tx_frame.data[3] = rv_type_convert.buf[3];

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

// Transmit MIT-mDme control(hybrid) package. Called in canTask.
void EvoMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    uint16_t p, v, kp, kd, t;

    f_p -= motor_zero_offset_;
    f_p = limit(f_p, -limit_param_.PosMax, limit_param_.PosMax);
    f_v = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(f_kp, 0.0f, limit_param_.OKpMax);
    f_kd = limit(f_kd, 0.0f, limit_param_.OKdMax);
    f_t = limit(f_t, -limit_param_.TauMax, limit_param_.TauMax);
    
    p = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), bitmax<uint16_t>(16));
    v = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), bitmax<uint16_t>(12));
    kp = range_map(f_kp, 0.0f, limit_param_.OKpMax, uint16_t(0), bitmax<uint16_t>(12));
    kd = range_map(f_kd, 0.0f, limit_param_.OKdMax, uint16_t(0), bitmax<uint16_t>(12));
    t = range_map(f_t, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), bitmax<uint16_t>(12));

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = p >> 8;
        tx_frame.data[1] = p & 0xFF;
        tx_frame.data[2] = v >> 4;
        tx_frame.data[3] = ((v & 0x0F) << 4) | (kp >> 8);
        tx_frame.data[4] = kp & 0xFF;
        tx_frame.data[5] = kd >> 4;
        tx_frame.data[6] = ((kd & 0x0F) << 4) | (t >> 8);
        tx_frame.data[7] = t & 0xFF;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
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
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) {
    if (!f_p || !f_v || !f_kp || !f_kd || !f_t) {
        return;
    }
    if (comm_type_ != CommType::CANFD) {
        motor_mit_cmd(*f_p, *f_v, *f_kp, *f_kd, *f_t);
        return;
    }
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    canfd_frame tx_frame;
    tx_frame.can_id = EVOFD_MIT_ID;
    tx_frame.len = 64;
    tx_frame.flags = CANFD_BRS;

    for (uint8_t slot = 0; slot < 8; ++slot) {
        uint8_t* base = &tx_frame.data[slot * 8];
        base[0] = 0x7F;
        base[1] = 0xFF;
        base[2] = 0x7F;
        base[3] = 0xF0;
        base[4] = 0x00;
        base[5] = 0x00;
        base[6] = 0x07;
        base[7] = 0xFF;
    }

    std::lock_guard<std::mutex> lock(bus_registry_mutex_);
    auto it = bus_registry_.find(can_interface_);
    if (it != bus_registry_.end()) {
        for (EvoMotorDriver* motor : it->second) {
            if (!motor || motor->motor_index_ >= 8) {
                continue;
            }
            const uint8_t slot = motor->motor_index_;
            
            float p_f, v_f, kp_f, kd_f, t_f;
            uint16_t p, v, kp, kd, t;

            p_f = limit(f_p[slot] - static_cast<float>(motor->motor_zero_offset_), -motor->limit_param_.PosMax, motor->limit_param_.PosMax);
            v_f = limit(f_v[slot], -motor->limit_param_.SpdMax, motor->limit_param_.SpdMax);
            kp_f = limit(f_kp[slot], 0.0f, motor->limit_param_.OKpMax);
            kd_f = limit(f_kd[slot], 0.0f, motor->limit_param_.OKdMax);
            t_f = limit(f_t[slot], -motor->limit_param_.TauMax, motor->limit_param_.TauMax);

            p = range_map(p_f, -motor->limit_param_.PosMax, motor->limit_param_.PosMax, uint16_t(0), uint16_t(0xFFFF));
            v = range_map(v_f, -motor->limit_param_.SpdMax, motor->limit_param_.SpdMax, uint16_t(0), uint16_t(0x0FFF));
            kp = range_map(kp_f, 0.0f, motor->limit_param_.OKpMax, uint16_t(0), uint16_t(0x0FFF));
            kd = range_map(kd_f, 0.0f, motor->limit_param_.OKdMax, uint16_t(0), uint16_t(0x0FFF));
            t = range_map(t_f, -motor->limit_param_.TauMax, motor->limit_param_.TauMax, uint16_t(0), uint16_t(0x0FFF));

            uint8_t* base = &tx_frame.data[slot * 8];
            base[0] = (p >> 8) & 0xFF;
            base[1] = p & 0xFF;
            base[2] = (v >> 4) & 0xFF;
            base[3] = ((v & 0x0F) << 4) | ((kp >> 8) & 0x0F);
            base[4] = kp & 0xFF;
            base[5] = (kd >> 4) & 0xFF;
            base[6] = ((kd & 0x0F) << 4) | ((t >> 8) & 0x0F);
            base[7] = t & 0xFF;
        }
    }
    canfd_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    if (comm_type_ == CommType::CAN) write_register_evo(11, 0x02);
    motor_control_mode_ = motor_control_mode;
}

void EvoMotorDriver::set_motor_id(uint8_t old_id, uint8_t new_id) {
    if (old_id == new_id) {
        logger_->warn("Skipping ID set: Old and New ID are identical ({})", old_id);
        return;
    }

    logger_->info("Changing Motor ID: {} -> {} (Interface: {})", old_id, new_id, can_interface_);

    // EVO Motor ID is register 36 (EVO_REG_MOTOR_ID, Int32)
    // Must send via current motor_id (old_id), writing new_id to register 36
    write_register_evo(EVO_REG_MOTOR_ID, static_cast<int32_t>(new_id));
    Timer::sleep_for(setup_sleep_time);
    logger_->info("Set ID command sent. Verify the new ID via bus query.");
}

void EvoMotorDriver::reset_motor_id() {
    logger_->info("Resetting Motor ID to 1 (Interface: {})", can_interface_);

    // Reset Motor ID to 1 by writing to register 36
    write_register_evo(EVO_REG_MOTOR_ID, static_cast<int32_t>(1));
    Timer::sleep_for(setup_sleep_time);
    logger_->info("Reset ID command sent. Verify the new ID via bus query.");
}

void EvoMotorDriver::set_motor_zero_evo() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = EVO_CMD_SET_ZERO;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
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
        tx_frame.data[7] = EVO_CMD_SET_ZERO;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::clear_motor_error_evo() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = EVO_CMD_DISABLE;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
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
        tx_frame.data[7] = EVO_CMD_DISABLE;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::write_register_evo(uint8_t rid, float value) {
    uint8_t* vbuf = (uint8_t*)&value;

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = EVO_CMD_START_FLASH;
        tx_frame.data[1] = rid;
        tx_frame.data[2] = vbuf[0];
        tx_frame.data[3] = vbuf[1];
        tx_frame.data[4] = vbuf[2];
        tx_frame.data[5] = vbuf[3];
        tx_frame.data[6] = EVO_CMD_WRITE_FLASH;
        tx_frame.data[7] = EVO_CMD_END_FLASH;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = EVO_CMD_START_FLASH;
        tx_frame.data[1] = rid;
        tx_frame.data[2] = vbuf[0];
        tx_frame.data[3] = vbuf[1];
        tx_frame.data[4] = vbuf[2];
        tx_frame.data[5] = vbuf[3];
        tx_frame.data[6] = EVO_CMD_WRITE_FLASH;
        tx_frame.data[7] = EVO_CMD_END_FLASH;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::write_register_evo(uint8_t index, int32_t value) {
    uint8_t* vbuf;
    vbuf = (uint8_t*)&value;

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;
        
        uint8_t* vbuf = (uint8_t*)&value;
        tx_frame.data[0] = EVO_CMD_START_FLASH;
        tx_frame.data[1] = index;
        tx_frame.data[2] = vbuf[0];
        tx_frame.data[3] = vbuf[1];
        tx_frame.data[4] = vbuf[2];
        tx_frame.data[5] = vbuf[3];
        tx_frame.data[6] = EVO_CMD_WRITE_FLASH;
        tx_frame.data[7] = EVO_CMD_END_FLASH;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = EVO_CMD_START_FLASH;
        tx_frame.data[1] = index;
        tx_frame.data[2] = *vbuf;
        tx_frame.data[3] = *(vbuf + 1);
        tx_frame.data[4] = *(vbuf + 2);
        tx_frame.data[5] = *(vbuf + 3);
        tx_frame.data[6] = EVO_CMD_WRITE_FLASH;
        tx_frame.data[7] = EVO_CMD_END_FLASH;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::save_register_evo() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;
        
        int offset = motor_index_ * 8;
        tx_frame.data[0] = EVO_CMD_START_FLASH;
        tx_frame.data[1] = 0x00;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = 0x00;
        tx_frame.data[5] = 0x00;
        tx_frame.data[6] = EVO_CMD_SAVE_FLASH;
        tx_frame.data[7] = EVO_CMD_END_FLASH;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.can_dlc = 0x08;
        
        tx_frame.data[0] = EVO_CMD_START_FLASH;
        tx_frame.data[1] = 0x00;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = 0x00;
        tx_frame.data[5] = 0x00;
        tx_frame.data[6] = EVO_CMD_SAVE_FLASH;
        tx_frame.data[7] = EVO_CMD_END_FLASH;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::refresh_motor_status() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = EVO_CMD_ENABLE;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
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
        tx_frame.data[7] = EVO_CMD_ENABLE;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::clear_motor_error() {
    clear_motor_error_evo();
}
