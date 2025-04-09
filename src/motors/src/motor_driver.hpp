#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

// #include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include <stdint.h>
#include <string.h>

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "SocketCAN.hpp"
#include "iostream"
#include "timer.hpp"
#include "utils.hpp"
using std::vector;
class MotorDriver {
   public:
    enum MotorControlMode_e {
        NONE = 0,
        MIT = 1,
        POS = 2,
        SPD = 3,
    };
    enum MotorErrorType_e {
        NONE_ERROR = 0,
        OVER_CURRENT = 1,
        OVER_TEMPERATURE = 2,
        COMMUNICATION_ERROR = 3,
    };

    static constexpr double judgment_accuracy_threshold = 1e-2;
    static const int normal_sleep_time = 5;
    static const int setup_sleep_time = 500;

    static constexpr float joint_lower_bounder_[7] = {-M_PI * 180 / 180,
                                                      -M_PI * 170 / 180,
                                                      -M_PI * 5 / 180,
                                                      -M_PI * 148 / 180,
                                                      -M_PI * 100 / 180,
                                                      -M_PI * 179 / 180,
                                                      -10};
    static constexpr float joint_upper_bounder_[7] = {+M_PI * 120 / 180,
                                                      +M_PI * 10 / 180,
                                                      +M_PI * 180 / 180,
                                                      +M_PI * 148 / 180,
                                                      +M_PI * 100 / 180,
                                                      +M_PI * 179 / 180,
                                                      +10};

    MotorDriver();
    virtual ~MotorDriver() = default;

    static std::shared_ptr<MotorDriver> MotorCreate(uint16_t motor_id, const char* interface,
                                                    const std::string type = std::string("OD"));

    virtual void CanRxMsgCallback(const can_frame& rx_frame) = 0;
    /**
     * @brief Locks the motor to prevent movement.
     *
     * This function locks the motor to prevent any movement.
     * Once locked, the motor will not respond to commands for movement.
     */
    virtual void MotorLock() = 0;

    /**
     * @brief Unlocks the motor to allow movement.
     *
     * This function unlocks the motor to enable movement.
     * After unlocking, the motor can respond to movement commands.
     */
    virtual void MotorUnlock() = 0;

    /**
     * @brief Initializes the motor.
     *
     * This function initializes the motor for operation.
     * It performs necessary setup and configuration for motor control.
     *
     * @return True if motor initialization is successful; otherwise, false.
     */
    virtual uint8_t MotorInit() = 0;

    /**
     * @brief Deinitializes the motor.
     *
     * This function deinitializes the motor.
     * It performs cleanup and releases resources associated with motor control.
     */
    virtual void MotorDeInit() = 0;

    /**
     * @brief Sets the motor position to zero.
     *
     * This function sets the current motor position to zero.
     * It establishes a new reference point for position measurement.
     *
     * @return True if setting motor position to zero is successful; otherwise, false.
     */
    virtual bool MotorSetZero() = 0;

    virtual bool MotorWriteFlash() = 0;

    virtual std::vector<double> MotorBoundary() {
        return std::vector<double>{joint_lower_bounder_[motor_id_ - 1], joint_upper_bounder_[motor_id_ - 1]};
    };

    /**
     * @brief Requests motor parameters based on a specific command.
     *
     * This function sends a request to retrieve specific parameters from the motor.
     * The parameter to be retrieved is identified by the `param_cmd` argument.
     *
     * @param param_cmd The command code specifying which parameter to retrieve.
     */
    virtual void MotorGetParam(uint8_t param_cmd) = 0;
    // to enum and union

    /**
     * @brief Commands the motor to move to a specified position at a specified speed.
     *
     * This function is responsible for commanding the motor to move to a desired position
     * with a specified speed.
     *
     * @param pos The target position to move the motor to.
     * @param spd The speed at which the motor should move to the target position.
     * @param ignore_limit If true, ignores any position limits that may be set.
     */
    virtual void MotorPosModeCmd(float pos, float spd, bool ignore_limit = false) = 0;

    /**
     * @brief Commands the motor to rotate at a specified speed.
     *
     * This function commands the motor to rotate at the specified speed.
     *
     * @param spd The speed at which the motor should rotate.
     */
    virtual void MotorSpdModeCmd(float spd) = 0;

    /**
     * @brief Commands the motor to operate in impedance mode with specific parameters.
     *
     * This function sets the motor to operate in an impedance control mode, where
     * it applies force based on the provided parameters.
     *
     * @param f_p Proportional force value.
     * @param f_v Velocity-based force value.
     * @param f_kp Proportional stiffness coefficient.
     * @param f_kd Damping coefficient.
     * @param f_t Desired torque value.
     */
    virtual void MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) = 0;

    /**
     * @brief Sets the position control parameters (proportional gain and derivative gain) for the motor.
     *
     * This function configures the position control parameters (PID gains) for the motor.
     *
     * @param kp Proportional gain for position control.
     * @param kd Derivative gain for position control.
     */
    virtual void MotorSetPosParam(float kp, float kd) = 0;

    /**
     * @brief Sets the speed control parameters (proportional gain and integral gain) for the motor.
     *
     * This function configures the speed control parameters (PI gains) for the motor.
     *
     * @param kp Proportional gain for speed control.
     * @param ki Integral gain for speed control.
     */
    virtual void MotorSetSpdParam(float kp, float ki) = 0;

    /**
     * @brief Sets the filter parameters for position and speed control.
     *
     * This function configures the filter parameters used in position and speed control.
     *
     * @param position_kd_filter Filter coefficient for position control derivative term.
     * @param kd_spd Filter coefficient for speed control derivative term.
     */
    virtual void MotorSetFilterParam(float position_kd_filter, float kd_spd) = 0;

    /**
     * @brief Sets the ID of the motor.
     *
     * This function assigns a unique identifier (ID) to the motor.
     *
     * @param motor_id The ID of the motor.
     */
    virtual void set_motor_id(uint8_t motor_id) = 0;

    /**
     * @brief Sets the control mode for the motor.
     *
     * This function specifies the control mode for the motor.
     *
     * @param motor_control_mode The control mode to be set for the motor.
     */
    virtual void set_motor_control_mode(uint8_t motor_control_mode) = 0;

    /**
     * @brief Retrieves the count of responses received from the motor.
     *
     * This function returns the number of responses received from the motor.
     *
     * @return The count of responses received from the motor.
     */
    virtual int get_response_count() const = 0;
    virtual void refresh_motor_status() = 0;

    // to get torque
    // to get error code

    virtual void MotorResetID() = 0;

    bool MotorCurrentDetect();
    bool MotorCommunicationDetect();
    bool MotorTemperatureDetect();
    bool MotorErrorDetect();
    void MotorErrorModeCmd();

    /**
     * @brief Retrieves the ID of the motor.
     *
     * This function returns the unique identifier (ID) of the motor.
     *
     * @return The ID of the motor.
     */
    virtual uint8_t get_motor_id() { return motor_id_; }

    /**
     * @brief Retrieves the control mode of the motor.
     *
     * This function returns the current control mode of the motor.
     *
     * @return The control mode of the motor.
     */
    virtual uint8_t get_motor_control_mode() { return motor_control_mode_; }

    /**
     * @brief Retrieves the error ID associated with the motor.
     *
     * This function returns the error ID that indicates any error condition of the motor.
     *
     * @return The error ID of the motor.
     */
    virtual uint8_t get_error_id() { return error_id_; }

    /**
     * @brief Retrieves the timeout value configured for the motor.
     *
     * This function returns the timeout value set for the motor operations.
     *
     * @return The timeout value in milliseconds.
     */
    virtual uint32_t get_timeout() { return timeout_; }

    /**
     * @brief Retrieves the gear ratio of the motor.
     *
     * This function returns the gear ratio of the motor.
     *
     * @return The gear ratio of the motor.
     */
    virtual float get_gear_ratio() { return gear_ratio_; }

    /**
     * @brief Retrieves the result of writing parameters to the motor.
     *
     * This function returns the result of writing parameters to the motor.
     *
     * @return True if writing parameters was successful; otherwise, false.
     */
    virtual bool get_write_para_res() { return write_para_res_; }

    /**
     * @brief Retrieves the current position of the motor.
     *
     * This function returns the current position of the motor.
     *
     * @return The current position of the motor.
     */
    virtual float get_motor_pos() { return motor_pos_; }

    /**
     * @brief Retrieves the current speed of the motor.
     *
     * This function returns the current speed of the motor.
     *
     * @return The current speed of the motor.
     */
    virtual float get_motor_spd() { return motor_spd_; }

    /**
     * @brief Retrieves the current current (electric current) of the motor.
     *
     * This function returns the electric current flowing through the motor.
     *
     * @return The current (electric current) of the motor.
     */
    virtual float get_motor_current() { return motor_current_; }

    /**
     * @brief Retrieves the error ID associated with the motor.
     *
     * This function returns the error ID that indicates any error condition of the motor.
     *
     * @return The error ID of the motor.
     */
    virtual float get_motor_error_id() { return error_id_; }

    /**
     * @brief Retrieves the temperature of the motor.
     *
     * This function returns the current temperature of the motor.
     *
     * @return The temperature of the motor.
     */
    virtual float get_motor_temperature() { return motor_temperature_; }

    /**
     * @brief Retrieves the acceleration of the motor.
     *
     * This function returns the current acceleration of the motor.
     *
     * @return The acceleration of the motor.
     */
    virtual float get_motor_acceleration() { return motor_acceleration_; }
    virtual float get_motor_kp_spd() { return motor_kp_spd; }
    virtual float get_motor_ki_spd() { return motor_ki_spd; }
    virtual float get_motor_kd_spd() { return motor_kd_spd; }
    virtual float get_motor_kp_pos() { return motor_kp_pos; }
    virtual float get_motor_ki_pos() { return motor_ki_pos; }
    virtual float get_motor_kd_pos() { return motor_kd_pos; }
    virtual void set_max_current(float max_current) { max_current_ = max_current; }
    virtual float get_max_current() const { return max_current_; }
    bool get_write_para_res_and_clear() {
        bool res = write_para_res_;
        write_para_res_ = false;
        return res;
    }
    static uint8_t motor_error_type_;

   protected:
    std::shared_ptr<spdlog::logger> logger_;
    uint16_t motor_id_;
    float max_current_ = 15.f;

    uint8_t motor_control_mode_;  // 0:none 1:pos 2:spd 3:mit

    uint8_t error_id_ = 0;
    uint32_t timeout_ = 0;    // dm 8 od 0x12
    float gear_ratio_ = 0.f;  // dm 11 od 0x0D
    bool write_para_res_ = false;

    float motor_pos_ = 0.f;      // od 1
    float motor_spd_ = 0.f;      // od 2
    float motor_current_ = 0.f;  // od 3
    float motor_temperature_ = 0.f;
    float motor_acceleration_ = 0.f;  // od 5 dm 3

    float motor_kp_spd = 0.f;  // dm 16 速度环比例参数
                               // od 6
    float motor_ki_spd = 0.f;  // dm 17 速度环积分参数
                               // od 7
    float motor_kd_spd = 0.f;  // od 11 速度环微分参数
                               // dm 1 扭矩系数
    float motor_kp_pos = 0.f;  // dm 18 位置环比例参数
                               // od 8
    float motor_ki_pos = 0.f;  // dm 19 位置环积分参数
                               // od 0x0C 位置环积分参数
    float motor_kd_pos = 0.f;  // od 9 位置环微分参数
                               // dm 22 (位置环)阻尼比

    uint16_t heartbeat_detect_counter_;
};

using union32_t = union Union32 {
    float f;
    int32_t i;
    uint32_t u;
    uint8_t buf[4];
};

#endif
