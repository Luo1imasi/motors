# roboto_motors

Roboto 平台电机驱动库，提供 C++ 和 Python SDK。支持 **DM（达妙）**、**EVO**、**Xynova（CAN-FD）**、**LeadRobot** 电机控制器，通过 SocketCAN 通信。

## 架构

```
roboto_motors/
├── include/
│   └── motor_driver.hpp           # 抽象基类 + 工厂模式
├── src/
│   ├── motor_driver.cpp           # 工厂方法: create_motor()
│   ├── pybind_module.cpp          # Python 绑定 (motors_py)
│   ├── utils.hpp                  # 工具函数 (limit, range_map, bitmax, Timer)
│   ├── drivers/
│   │   ├── dm/                    # 达妙电机驱动 — CAN
│   │   ├── evo/                   # EVO 电机驱动 — CAN
│   │   ├── xyn/                   # Xynova 电机驱动 — CAN-FD（29 位扩展 ID）
│   │   └── lro/                   # LeadRobot 电机驱动 — CAN
│   └── protocol/
│       ├── can/
│       │   └── socket_can.hpp/.cpp      # SocketCAN + CAN-FD（单例，无锁 TX 队列，SCHED_FIFO）
│       └── ethercat/                    # 预留
```

## 支持电机

| 类型 | 电机型号 | 接口 | 协议 | 控制模式 |
|------|---------|------|------|----------|
| DM（达妙） | DM4340P-48V, DM10010L-48V | CAN | 标准 CAN | MIT, 位置, 速度 |
| EVO | EVO431040, EVO811825, EVO811832 | CAN | 标准 CAN | MIT |
| XYN（Xynova） | — | CAN-FD | 29 位扩展 ID | 位置, 速度, 力矩 |
| LRO（LeadRobot） | — | CAN | 标准 CAN | MIT, 位置, 速度, 电流 |

## API

### 工厂方法

```cpp
auto motor = MotorDriver::create_motor(
    motor_id,           // uint16_t: CAN 节点 ID
    interface_type,     // string: "can"
    interface,          // string: CAN 接口名, 例如 "can0"
    motor_type,         // string: "DM" / "EVO" / "XYN" / "LRO"
    motor_model,        // int: 型号枚举 (如 DM4340P_48V=0, EVO431040=0)
    master_id_offset,   // uint16_t: 主站 ID 偏移 (仅 DM, 默认 0)
    motor_zero_offset   // double: 零位偏移, rad (默认 0.0)
);
```

### 控制方法

| 方法 | 参数 | 说明 |
|------|------|------|
| `init_motor()` | — | 初始化并使能电机 |
| `deinit_motor()` | — | 失能并释放电机 |
| `lock_motor()` | — | 锁定电机（禁止运动） |
| `unlock_motor()` | — | 解锁电机（允许运动） |
| `motor_mit_cmd(pos, vel, kp, kd, torque)` | float×5 | MIT 阻抗控制 |
| `motor_pos_cmd(pos, spd, ignore_limit)` | float, float, bool | 位置控制 (rad, rad/s) |
| `motor_spd_cmd(spd)` | float | 速度控制 (rad/s) |
| `set_motor_zero()` | — | 设置当前位置为零位 |
| `write_motor_flash()` | — | 保存参数到 Flash |
| `set_motor_control_mode(mode)` | uint8_t | 设置模式: MIT=1, POS=2, SPD=3 |
| `clear_motor_error()` | — | 清除错误状态 |
| `reset_motor_id()` | — | 重置电机 ID |

### 反馈数据

| 方法 | 返回值 | 单位 |
|------|--------|------|
| `get_motor_pos()` | float | rad |
| `get_motor_spd()` | float | rad/s |
| `get_motor_current()` | float | A |
| `get_motor_temperature()` | float | °C |
| `get_motor_id()` | uint8_t | — |
| `get_motor_control_mode()` | uint8_t | — |
| `get_error_id()` | uint8_t | — |
| `get_response_count()` | int | — |

## 编译

依赖: `spdlog`, `fmt`, `Boost`, `Eigen3`, `pybind11`, `ament_cmake` (ROS 2)

```bash
colcon build --packages-select roboto_motors
```

## 使用方法

### Python

```python
from motors_py import MotorDriver, MotorControlMode

# 达妙电机
motor = MotorDriver.create_motor(
    motor_id=0x01,
    interface_type="can",
    interface="can0",
    motor_type="DM",
    motor_model=0        # DM4340P_48V
)

# LeadRobot 电机
motor = MotorDriver.create_motor(
    motor_id=0x01,
    interface_type="can",
    interface="can0",
    motor_type="LRO",
    motor_model=0
)

# Xynova CAN-FD 电机
motor = MotorDriver.create_motor(
    motor_id=0x01,
    interface_type="can",
    interface="can0",
    motor_type="XYN",
    motor_model=0
)

# 初始化
motor.init_motor()

# MIT 阻抗控制
motor.set_motor_control_mode(MotorControlMode.MIT)
motor.motor_mit_cmd(0.0, 0.0, 10.0, 1.0, 0.0)

# 位置控制
motor.set_motor_control_mode(MotorControlMode.POS)
motor.motor_pos_cmd(1.57, 3.14)  # 90° @ ~180°/s

# 速度控制
motor.set_motor_control_mode(MotorControlMode.SPD)
motor.motor_spd_cmd(3.14)  # ~180°/s

# 读取反馈
pos = motor.get_motor_pos()           # rad
spd = motor.get_motor_spd()           # rad/s
cur = motor.get_motor_current()       # A
temp = motor.get_motor_temperature()  # °C

# 关闭
motor.deinit_motor()
```

### C++

```cpp
#include "motor_driver.hpp"

auto motor = MotorDriver::create_motor(0x01, "can", "can0", "DM", 0);
motor->init_motor();

// MIT 控制
motor->set_motor_control_mode(MotorDriver::MIT);
motor->motor_mit_cmd(0.0f, 0.0f, 10.0f, 1.0f, 0.0f);

// 读取反馈
float pos = motor->get_motor_pos();
float spd = motor->get_motor_spd();

motor->deinit_motor();
```

## CAN 接口配置

```bash
# 标准 CAN (DM / EVO / LRO)
sudo ip link set can0 up type can bitrate 1000000

# CAN-FD (Xynova)
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
```

## 许可证

Apache License 2.0
