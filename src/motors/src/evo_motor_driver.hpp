#pragma once

#include <atomic>
#include <string>

#include "motor_driver.hpp"
enum EVOError {
    EVO_NO_ERROR = 0x00,
    EVO_OVER_VOLTAGE = 0x01,
    EVO_UNDER_VOLTAGE = 0x02,
    EVO_OVER_CURRENT = 0x03,
    EVO_MOS_OVER_TEMP = 0x04,
    EVO_COIL_OVER_TEMP = 0x05,
    EVO_COMM_LOST = 0x06,
    EVO_OVERLOAD = 0x07,
    EVO_ENCODER_ERROR = 0x08,
    EVO_UNKNOWN_ERROR = 0xFF
};

enum EVO_Motor_Model { 
    REVO_4310 = 0,      ///< REVO standard motor (12-bit Kd, OKD_MAX=50.0)
    ENCOS_8108 = 1,     ///< ENCOS high-performance motor (9-bit Kd, OKD_MAX=5.0)
    EVO_Num_Of_Model 
};

enum EVO_REG {
    EVO_CMD_MOTOR_MODE = 0xFC,      ///< Enable motor mode
    EVO_CMD_RESET_MODE = 0xFD,      ///< Reset motor and clear errors
    EVO_CMD_SET_ZERO = 0xFE,        ///< Set current position as zero point
    EVO_CMD_WRITE_FLASH = 0x12,     ///< Write parameter to flash memory
    EVO_CMD_READ_FLASH = 0x13,      ///< Read parameters from flash memory
    EVO_CMD_REBOOT = 0xFE           ///< Reboot motor (make flash parameters effective)
};

enum EVO_Flash_Param {
    EVO_PARAM_Q_MAX = 0x00,         ///< Maximum position limit
    EVO_PARAM_Q_MIN = 0x01,         ///< Minimum position limit
    EVO_PARAM_DQ_MAX = 0x02,        ///< Maximum velocity limit
    EVO_PARAM_DQ_MIN = 0x03,        ///< Minimum velocity limit
    EVO_PARAM_TAU_MAX = 0x04,       ///< Maximum torque/current limit
    EVO_PARAM_TAU_MIN = 0x05,       ///< Minimum torque/current limit
    EVO_PARAM_OKP_MAX = 0x06,       ///< Maximum outer Kp
    EVO_PARAM_OKP_MIN = 0x07,       ///< Minimum outer Kp
    EVO_PARAM_OKD_MAX = 0x08,       ///< Maximum outer Kd
    EVO_PARAM_OKD_MIN = 0x09,       ///< Minimum outer Kd
    EVO_PARAM_IKP_MAX = 0x0A,       ///< Maximum inner Kp
    EVO_PARAM_IKP_MIN = 0x0B,       ///< Minimum inner Kp
    EVO_PARAM_IKI_MAX = 0x0C,       ///< Maximum inner Ki
    EVO_PARAM_IKI_MIN = 0x0D,       ///< Minimum inner Ki
    EVO_PARAM_CUR_MAX = 0x0E,       ///< Maximum current
    EVO_PARAM_CUR_MIN = 0x0F        ///< Minimum current
};

typedef struct {
    float PosMax;       ///< Maximum position limit (rad)
    float SpdMax;       ///< Maximum velocity limit (rad/s)
    float TauMax;       ///< Maximum torque/current limit (N·m or A)
    float OKP_Max;      ///< Maximum outer-loop proportional gain
    float OKD_Max;      ///< Maximum outer-loop derivative gain
    float CUR_Max;      ///< Maximum current limit (A)
} EVO_Limit_Param;

class EvoMotorDriver : public MotorDriver {
   public:
    EvoMotorDriver(uint16_t motor_id, std::string can_interface, uint16_t master_id_offset,
                   EVO_Motor_Model motor_model);
    ~EvoMotorDriver();

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
    virtual void MotorResetID() override {};
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual int get_response_count() const { 
        return response_count_; 
    }
    virtual void refresh_motor_status() override;
   private:
    std::atomic<int> response_count_{0};
    const float KpMin = 0.0f;
    const float KpMax = 500.0f;
    const float KdMin = 0.0f;
    float KdMax ;
    EVO_Motor_Model motor_model_;
    EVO_Limit_Param limit_param_;
    std::atomic<uint8_t> mos_temperature_{0};
    void EvoMotorSetZero();
    void EvoMotorClearError();
    void EvoWriteRegister(uint16_t index, uint8_t subindex, int32_t value);
    void EvoReadRegister(uint16_t index, uint8_t subindex);
    void EvoSaveRegister(uint8_t rid);
    virtual void CanRxMsgCallback(const can_frame& rx_frame) override;
    std::shared_ptr<SocketCAN> can_;
    
    inline int getKdBitWidth() const {
        return (motor_model_ == REVO_4310) ? 12 : 9;
    }
    
    inline float getKdMax() const {
        return limit_param_.OKD_Max;
    }
};