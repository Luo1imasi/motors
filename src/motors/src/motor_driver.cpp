

#include "motor_driver.hpp"

#include "dm_motor_driver.hpp"

uint8_t MotorDriver::motor_error_type_ = NONE_ERROR;

MotorDriver::MotorDriver() {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
    logger_ = setup_logger(sinks);
}
std::shared_ptr<MotorDriver> MotorDriver::MotorCreate(uint16_t motor_id, const char* interface,
                                                      const std::string type, uint16_t master_id_offset) {
    if (type == "DM") {
        return std::make_shared<DmMotorDriver>(motor_id, interface, master_id_offset);
    } else {
        throw std::runtime_error("Motor type not supported");
    }
}

bool MotorDriver::MotorCurrentDetect() {
    static int detect_current_counter[6] = {0};
    if (get_motor_current() < -10.0f) {
        return false;
    }
    if (abs(get_motor_current()) > 25.0f) {
        detect_current_counter[motor_id_ - 1]++;
    } else {
        detect_current_counter[motor_id_ - 1] = 0;
    }
    if (detect_current_counter[motor_id_ - 1] > 60) {
        logger_->info("motor {} is over current\n", motor_id_);
        motor_error_type_ = OVER_CURRENT;
        return true;
    }
    return false;
}

bool MotorDriver::MotorCommunicationDetect() {
    static int detect_communication_counter[6] = {0};
    if (heartbeat_detect_counter_ != 0) {
        detect_communication_counter[motor_id_ - 1]++;
    } else {
        heartbeat_detect_counter_++;
        detect_communication_counter[motor_id_ - 1] = 0;
    }
    if (detect_communication_counter[motor_id_ - 1] > 1e5) {
        logger_->warn("motor {} is communication error\n", motor_id_);
        motor_error_type_ = COMMUNICATION_ERROR;
        return true;
    }
    return false;
}

bool MotorDriver::MotorTemperatureDetect() {
    static int detect_temperature_counter[6] = {0};
    if (motor_temperature_ > 80.0f) {
        detect_temperature_counter[motor_id_ - 1]++;
    } else {
        detect_temperature_counter[motor_id_ - 1] = 0;
    }
    if (detect_temperature_counter[motor_id_ - 1] > 60) {
        logger_->warn("motor {} is over temperature\n", motor_id_);
        motor_error_type_ = OVER_TEMPERATURE;
        return true;
    }
    return false;
}

bool MotorDriver::MotorErrorDetect() {
    return MotorCurrentDetect() || MotorCommunicationDetect() || MotorTemperatureDetect();
}

void MotorDriver::MotorErrorModeCmd() {
    switch (motor_error_type_) {
        case OVER_CURRENT:
            MotorPosModeCmd(0.0f, 0.5f);
            break;
        case OVER_TEMPERATURE:
            MotorPosModeCmd(0.0f, 0.5f);
            break;
        case COMMUNICATION_ERROR:
            MotorPosModeCmd(get_motor_pos(), 0.5f);
            break;
        default:
            break;
    }
}

// MotorDriver* motor[6];
