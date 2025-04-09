#ifndef DM_MOTOR_DRIVER_HPP
#define DM_MOTOR_DRIVER_HPP
#include <mutex>
#include <string>

#include "motor_driver.hpp"
enum DMError {
    DM_DOWN = 0x00,
    DM_UP = 0x01,
    OVER_VOLT = 0x08,
    UNDER_VOLT = 0x09,
    OVER_CURRENT = 0x0A,
    MOS_OVER_TEMP = 0x0B,
    COIL_OVER_TEMP = 0x0C,
    LOST_CONN = 0x0D,
    OVER_LOAD = 0x0E,
};

class DmMotorDriver : public MotorDriver {
   public:
    DmMotorDriver(uint16_t motor_id, std::string can_interface);
    ~DmMotorDriver();

    virtual void MotorLock() override;
    virtual void MotorUnlock() override;
    virtual uint8_t MotorInit() override;
    virtual void MotorDeInit() override;
    virtual bool MotorSetZero() override;
    virtual bool MotorWriteFlash() override;

    virtual void MotorGetParam(uint8_t param_cmd) override;
    virtual void MotorPosModeCmd(float pos, float spd, bool ignore_limit) override;
    virtual void MotorSpdModeCmd(float spd) override;
    virtual void MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    virtual void MotorSetPosParam(float kp, float kd) override;
    virtual void MotorSetSpdParam(float kp, float ki) override;
    virtual void MotorSetFilterParam(float position_kd_filter, float kd_spd) override;
    virtual void MotorResetID() override {};
    virtual void set_motor_id(uint8_t motor_id) override;
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual int get_response_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return response_count;
    }
    virtual void refresh_motor_status() override;

   private:
    int response_count = 0;
    mutable std::mutex mutex_;
    bool param_cmd_flag_[30] = {false};
    // const std::string kFirmWareVersion = "3163";
    const float kKpMin = 0.0f;
    const float kKpMax = 500.0f;
    const float kKdMin = 0.0f;
    const float kKdMax = 5.0f;
    const float kIMin = -30.0f;
    const float kIMax = 30.0f;
    const float kPMax = 12.5f;
    const float kPMin = -12.5f;
    const float kSpdMin = -30.0f;
    const float kSpdMax = 30.0f;
    const float kTorqueMin = -10.0f;
    const float kTorqueMax = 10.0f;
    uint8_t master_id_ = 0;
    uint8_t mos_temperature_ = 0;
    float pos_max_;                  // dm 12 位置映射最大值
    float vel_max_;                  // dm 13 速度映射最大值
    float torque_max_;               // dm 14 扭矩映射最大值
    float current_bandwidth_;        // dm 15 电流环控制带宽
    float gear_torque_coefficient_;  // dm 21 齿轮扭矩系数
    uint8_t ctrl_mode_;              // dm 9 控制模式
    float under_voltage_;            // dm 0 欠压保护值
    float over_voltage_;             // dm 20 过压保护值
    float current_limit_;            // dm 2 限流值
    float dec_;                      // dm 4 减速度
    float max_speed_;                // dm 5 最大速度
    void DmMotorSetZero();
    void DmMotorClearError();
    void DmWriteRegister(uint8_t rid, float value);
    void DmWriteRegister(uint8_t rid, int32_t value);
    void DmSaveRegister(uint8_t rid);
    virtual void CanRxMsgCallback(const can_frame& rx_frame) override;
    std::shared_ptr<SocketCAN> can_;
};

#endif
