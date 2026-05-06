# roboto_motors

Motor driver library for the Roboto platform with C++ and Python SDK support. Supports **DM** (达妙), **EVO**, **Xynova** (CAN-FD), and **LeadRobot** motor controllers via SocketCAN.

## Architecture

```
roboto_motors/
├── include/
│   └── motor_driver.hpp           # Abstract base class with factory pattern
├── src/
│   ├── motor_driver.cpp           # Factory: create_motor()
│   ├── pybind_module.cpp          # Python bindings (motors_py)
│   ├── utils.hpp                  # Utility functions (limit, range_map, bitmax, Timer)
│   ├── drivers/
│   │   ├── dm/                    # DM (达妙) motor driver — CAN
│   │   ├── evo/                   # EVO motor driver — CAN
│   │   ├── xyn/                   # Xynova motor driver — CAN-FD (29-bit extended ID)
│   │   └── lro/                   # LeadRobot motor driver — CAN
│   └── protocol/
│       ├── can/
│       │   └── socket_can.hpp/.cpp      # SocketCAN + CAN-FD (singleton, lockfree TX, SCHED_FIFO)
│       └── ethercat/                    # Reserved
```

## Supported Motors

| Type | Motor Models | Interface | Protocol | Control Modes |
|------|-------------|-----------|----------|---------------|
| DM | DM4340P-48V, DM10010L-48V | CAN | Standard CAN | MIT, POS, SPD |
| EVO | EVO431040, EVO811825, EVO811832 | CAN | Standard CAN | MIT |
| XYN (Xynova) | — | CAN-FD | 29-bit Extended ID | POS, SPD, Torque |
| LRO (LeadRobot) | — | CAN | Standard CAN | MIT, POS, SPD, Current |

## API

### Factory

```cpp
auto motor = MotorDriver::create_motor(
    motor_id,           // uint16_t: CAN node ID
    interface_type,     // string: "can"
    interface,          // string: CAN interface name, e.g. "can0"
    motor_type,         // string: "DM" / "EVO" / "XYN" / "LRO"
    motor_model,        // int: model enum (e.g. DM4340P_48V=0, EVO431040=0)
    master_id_offset,   // uint16_t: master ID offset (DM only, default 0)
    motor_zero_offset   // double: zero offset in rad (default 0.0)
);
```

### Control Methods

| Method | Parameters | Description |
|--------|-----------|-------------|
| `init_motor()` | — | Initialize and enable motor |
| `deinit_motor()` | — | Disable and release motor |
| `lock_motor()` | — | Lock motor (prevent movement) |
| `unlock_motor()` | — | Unlock motor (allow movement) |
| `motor_mit_cmd(pos, vel, kp, kd, torque)` | float×5 | MIT impedance control |
| `motor_pos_cmd(pos, spd, ignore_limit)` | float, float, bool | Position control (rad, rad/s) |
| `motor_spd_cmd(spd)` | float | Speed control (rad/s) |
| `set_motor_zero()` | — | Set current position as zero |
| `write_motor_flash()` | — | Save parameters to flash |
| `set_motor_control_mode(mode)` | uint8_t | Set mode: MIT=1, POS=2, SPD=3 |
| `clear_motor_error()` | — | Clear error state |
| `reset_motor_id()` | — | Reset motor ID |

### Feedback

| Method | Return | Unit |
|--------|--------|------|
| `get_motor_pos()` | float | rad |
| `get_motor_spd()` | float | rad/s |
| `get_motor_current()` | float | A |
| `get_motor_temperature()` | float | °C |
| `get_motor_id()` | uint8_t | — |
| `get_motor_control_mode()` | uint8_t | — |
| `get_error_id()` | uint8_t | — |
| `get_response_count()` | int | — |

## Build

Requires: `spdlog`, `fmt`, `Boost`, `Eigen3`, `pybind11`, `ament_cmake` (ROS 2)

```bash
colcon build --packages-select roboto_motors
```

## Usage

### Python

```python
from motors_py import MotorDriver, MotorControlMode

# DM motor
motor = MotorDriver.create_motor(
    motor_id=0x01,
    interface_type="can",
    interface="can0",
    motor_type="DM",
    motor_model=0        # DM4340P_48V
)

# LeadRobot motor
motor = MotorDriver.create_motor(
    motor_id=0x01,
    interface_type="can",
    interface="can0",
    motor_type="LRO",
    motor_model=0
)

# Xynova CAN-FD motor
motor = MotorDriver.create_motor(
    motor_id=0x01,
    interface_type="can",
    interface="can0",
    motor_type="XYN",
    motor_model=0
)

# Initialize
motor.init_motor()

# MIT impedance control
motor.set_motor_control_mode(MotorControlMode.MIT)
motor.motor_mit_cmd(0.0, 0.0, 10.0, 1.0, 0.0)

# Position control
motor.set_motor_control_mode(MotorControlMode.POS)
motor.motor_pos_cmd(1.57, 3.14)  # 90° at ~180°/s

# Speed control
motor.set_motor_control_mode(MotorControlMode.SPD)
motor.motor_spd_cmd(3.14)  # ~180°/s

# Read feedback
pos = motor.get_motor_pos()           # rad
spd = motor.get_motor_spd()           # rad/s
cur = motor.get_motor_current()       # A
temp = motor.get_motor_temperature()  # °C

# Shutdown
motor.deinit_motor()
```

### C++

```cpp
#include "motor_driver.hpp"

auto motor = MotorDriver::create_motor(0x01, "can", "can0", "DM", 0);
motor->init_motor();

// MIT control
motor->set_motor_control_mode(MotorDriver::MIT);
motor->motor_mit_cmd(0.0f, 0.0f, 10.0f, 1.0f, 0.0f);

// Read feedback
float pos = motor->get_motor_pos();
float spd = motor->get_motor_spd();

motor->deinit_motor();
```

## CAN Interface Setup

```bash
# Standard CAN (DM / EVO / LRO)
sudo ip link set can0 up type can bitrate 1000000

# CAN-FD (Xynova)
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
```

## License

Apache License 2.0
