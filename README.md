# Meta Team - Infantry 2023
## Current project status
master![C/C++ CI](https://github.com/Meta-Team/Meta-Embedded/actions/workflows/release.yml/badge.svg?branch=master)

=> 请务必阅读 [Project Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki) :smiley:

=> Please read [Project Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki) :smiley:

ZJU-UIUC Meta 战队 RoboMaster 嵌入式程序工程，基于 ChibiOS 操作系统，使用 C 和 C++。
ZJU-UIUC Meta RoboMaster Team's Embedded Control Program, based on ChibiOS/RT. The program mainly use C and C++.

本工程包含步兵、工程、英雄、哨兵机器人源代码，适用于RoboMaster 开发板A型，通过条件编译控制编译目标。
The project contains the standard, engineer, hero and sentry vehicles' source code. This is compatible with RoboMaster Development Board Type A.
The code for each robot could be built by switching the target of CMake.


# 基本结构 | Basic Structures
更新日期：May 1, 2023
Updated: May 1, 2023

```
Meta-Embedded
├── cmsis            CMSIS DSP 计算库 | CMSIS DSP Library for Calculation
├── config           CMake 配置、OpenOCD 配置、辅助脚本等 | Configuration for CMake, OpenOCD and shell scripts.
├── dev              我们编写的源码文件 | Main source code.
├── os               ChibiOS 源码文件 | ChibiOS/RT code.
├── CMakeLists.txt   CMake 主配置文件 | Main configuration file for CMake.
└── README.md        此文件 | This file.
```

工程使用类实现封装和模块化，使用线程控制模块间的调用和信息传递。

The project use C++ class for encapsulation and modularization. The modules will be called by threads.

```
.
├── board_pin                           - STM32配置 | Configuration for STM32 -
│   ├── rm_board_2017
│   │     ├── board.c
│   │     ├── board.h
│   │     ├── board.mk
│   │     └── mcuconf.h
│   └── rm_board_2018_a
│         ├── board.c
│         ├── board.h
│         ├── board.mk
│         └── mcuconf.h
├── application                         - 应用层程序 | Higher level Applications -
│   ├───param_adjusts                       - 参数整调程序 | Parameter Adjust Programs -
│   │   ├───pa_chassis               
│   │   └───pa_infantry              
│   ├───unit_tests                      - 单元测试 | Unit Tests-
│   │   ├───ut_ahrs
│   │   ├───ut_buzzer
│   │   ├───ut_chassis
│   │   ├───ut_led
│   │   ├───ut_oled
│   │   ├───ut_referee_if
│   │   ├───ut_remote_if
│   │   ├───ut_sd_card
│   │   ├───ut_sentry_chassis
│   │   └───ut_usb_com
│   └───vehicles                        - 车辆配置文件（电机ID，PID参数等） | Vehicles Configuration -
│       ├───aerial
│       ├───engineer
│       ├───haptic_device
│       ├───hero
│       ├───infantry
│       └───sentry
├── interface                           - 较低层次的接口模块 | Low Level Interfaces -
│   ├── ahrs                                - IMU 模块 | IMU Module -
│   │   ├── ahrs.cpp                            - 综合运算模块 | Comprehensive Calculation Module -
│   │   ├── ahrs.h
│   │   ├── imu_math.hpp
│   │   ├── ist8310.cpp                         - IST8310 接口 | Interface for IST8310 -
│   │   ├── ist8310.h
│   │   ├── ist8310_reg.h
│   │   ├── mpu6500.cpp                         - MPU6500 陀螺仪接口 | Interface for MPU 6500 -
│   │   ├── mpu6500.h
│   │   └── mpu6500_reg.h
│   ├── can                                 - CAN总下相关接口 | Interfaces for CAN-BUS -
│   │   ├── can_interface.cpp                   - CAN总线收发程序 | CAN-BUS Tx/RX Module
│   │   ├── can_interface.h
│   │   ├── can_motor_feedback.cpp              - 大疆CAN电机接口 | DJI Motors' Feedback Class Using CAN-BUS -
│   │   ├── can_motor_feedback.h
│   │   ├── CANMotorIF.cpp                      - 大疆CAN电机整体接口 | Overall Interface for DJI Motors -
│   │   └── CANMotorIF.h
│   ├── usb_com                             - 虚拟串口 | STM32 Virtual COM port (CDC) -
│   │   ├── usb_serial_port.cpp                 - USB配置文件 | USB configuration file
│   │   ├── usb_serial_port.h
│   │   ├── VirtualCOMPort.cpp                  - 虚拟串口收发接口 | Interface for Virtual COM Port
│   │   └── VirtualCOMPort.h
│   └── ...                                 - 其他接口（可能会频繁更新） | Other Interfaces (Varies Over Time)
├── scheduler                           - 调度程序（包含控制算法） | Schedulers, contains thread performing control algorithms -
│   ├── buzzer_scheduler.cpp                - 蜂鸣器 | Buzzer
│   ├── buzzer_scheduler.h
│   ├── CANMotorController.cpp              - CAN电机调度程序 | Schedulers for CAN motors. Performing control algorithms (like PID)
│   ├── CANMotorController.h
│   └── ...                                 - 其他调度程序（近期更新） | Other Schedulers (Might Refactor Soon)
├── logic                               - 高级逻辑控制 | Higher Level Logic Control -
│   ├── HapticLG.cpp                        - 重力模拟器逻辑控制，包含开机校准以及多种模式控制 | Haptic device logic control, contains startup calibration and multiple control modes
│   ├── HapticLG.h
│   ├── chassis_logic.cpp                   - 麦克纳姆轮底盘控制程序，包含速度分解 | Control Algorithm for chassises with Mecanum wheel, including velocity decompose
│   ├── chassis_logic.h
│   └── ...                                 - 其他逻辑控制程序（近期更新） | Other high level logical control algorithms (Might Refactor Soon)
├── common                              - 通用组件 | Universal Components -
│   ├── common_macro.h
│   ├── CRC8.cpp
│   ├── CRC8.h
│   ├── CRC16.cpp
│   └── CRC16.h
├── module                              - 模块（将来可能与common合并） | common modules (might merge with common soon) -                       
├── chconf.h                            - ChibiOS 配置文件 | ChibiOS configuration
└── halconf.h                           - ChibiOS HAL 配置文件 | ChibiOS HAL configuraion
```

# 前置阅读资料 | Pre-request
* 工具链相关资料，参考 [Meta-Infantry Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki)
* Toolchain, [Meta-Infantry Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki)
* [CMake 构建系统简介](https://github.com/Meta-Team/Meta-Infantry/wiki/CMake-%E6%9E%84%E5%BB%BA%E7%B3%BB%E7%BB%9F%E7%AE%80%E4%BB%8B)
* CMake[Brief Introduction to CMake](https://github.com/Meta-Team/Meta-Infantry/wiki/CMake-%E6%9E%84%E5%BB%BA%E7%B3%BB%E7%BB%9F%E7%AE%80%E4%BB%8B)
* [Meta Team C++ Style Guide](https://github.com/Meta-Team/Meta-Infantry/wiki/Meta-Team-C---Style-Guide)