#!/usr/bin/env python3
import motors_py
import time

def example_can_motor():
    """使用CAN总线连接的电机示例"""
    print("=== CAN电机示例 ===")
    
    try:
        motor = motors_py.MotorDriver.create_motor(
            motor_id=24,
            interface_type="can",
            interface="can0",
            motor_type="EVO",
            motor_model=0,
        )
        print("电机创建成功！")
    except Exception as e:
        print(f"创建电机失败: {e}")
        return
    
    try:
        print("使能电机...")
        motor.init_motor()
        
        print("\n=== MIT模式控制示例 ===")
        motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)
        
        target_pos = 0.3
        target_vel = 0.0
        kp = 5.0
        kd = 1.0
        torque = 0.0
        
        motor.motor_mit_cmd(target_pos, target_vel, kp, kd, torque)
            
        # 读取电机状态
        pos = motor.get_motor_pos()
        vel = motor.get_motor_spd()
        current = motor.get_motor_current()
        temp = motor.get_motor_temperature()
        error_id = motor.get_error_id()
        
        print(f"位置: {pos:.4f} rad, 速度: {vel:.4f} rad/s, "
              f"电流: {current:.4f} A, 温度: {temp:.2f}°C, 错误码: {error_id}")
        time.sleep(1)
    except Exception as e:
        print(f"电机控制过程中出错: {e}")
    finally:
        motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)
        motor.motor_mit_cmd(0.0, 0.0, 0.0, 1.0, 0.0)
        
        print("失能电机...")
        motor.deinit_motor()


if __name__ == "__main__":
    example_can_motor()
