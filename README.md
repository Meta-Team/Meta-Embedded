# Meta Team - Infantry 2021

=> 请务必阅读 [Project Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki) :smiley:

=> Please read [Project Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki) :smiley:

ZJU-UIUC Meta 战队 RoboMaster 嵌入式程序工程，基于 ChibiOS 操作系统，使用 C 和 C++。

ZJU-UIUC Meta RoboMaster Team's Embedded Control Program, based on ChibiOS/RT. The program mainly use C and C++.

工程包含步兵、工程、英雄、哨兵机器人源代码，适用于 RM 2018 A 型、RM 2017 开发板，通过条件编译控制编译目标。

The project contains the standard, engineer, hero and sentry vehicles source code and is compatible with RM board 2018 A
and RM board 2017. With CMake, the code for each robot could be built by switching the target of CMake.


# 基本结构 | Basic Structures


更新日期：Nov.24, 2021
Updated Time: Nov.24, 2021

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
├── board                               - STM32配置 | Configuration for STM32 -
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
├── interface                           - 较低层次的接口模块 | Low Level Interfaces -
│   ├── ahrs                            - IMU 模块 | IMU Module -
│   │   ├── ahrs.cpp                      综合运算模块 | Comprehensive Calculation Module -
│   │   ├── ahrs.h
│   │   ├── imu_math.hpp
│   │   ├── ist8310.cpp                   IST8310 接口 | Interface for IST8310 -
│   │   ├── ist8310.h
│   │   ├── ist8310_reg.h
│   │   ├── mpu6500.cpp                   MPU6500 陀螺仪接口 | Interface for MPU 6500 -
│   │   ├── mpu6500.h
│   │   └── mpu6500_reg.h
│   ├── can                             - CAN总下相关接口 | Interfaces for CAN-BUS -
│   │   ├── can_interface.cpp             CAN总线收发程序 | CAN-BUS Tx/RX Module
│   │   ├── can_interface.h
│   │   ├── can_motor_feedback.cpp        大疆CAN电机接口 | DJI Motors' Feedback Class Using CAN-BUS -
│   │   ├── can_motor_feedback.h
│   │   ├── can_motor_interface.cpp       大疆CAN电机整体接口 | Overall Interface for DJI Motors -
│   │   └── can_motor_interface.h
│   ├── virtual_COM                     - 虚拟串口 | STM32 Virtual COM port (CDC) -
│   │   ├── usbconf.cpp                   USB配置文件 | USB configuration file
│   │   ├── usbconf.h
│   │   ├── VCP.cpp                       虚拟串口收发接口 | Interface for Virtual COM Port
│   │   └── VCP.h
│   └── ...                               其他接口（可能会频繁更新） | Other Interfaces (Varies Over Time)
├── scheduler                           - 调度程序（包含控制算法） | Schedulers, contains thread performing control algorithms -
│   ├── buzzer_scheduler.cpp              蜂鸣器 | Buzzer
│   ├── buzzer_scheduler.h
│   ├── can_motor_scheduler.cpp           CAN电机调度程序 | Schedulers for CAN motors. Performing control algorithms (like PID)
│   ├── can_motor_scheduler.h
│   └── ...                               其他调度程序（近期更新） | Other Schedulers (Might Refactor Soon)
├── logic                               - 高级逻辑控制 | Higher Level Logic Control -
│   ├── haptic_logic.cpp                  重力模拟器逻辑控制，包含开机校准以及多种模式控制 | Haptic device logic control, contains startup calibration and multiple control modes
│   ├── haptic_logic.h
│   ├── chassis_logic.cpp                 麦克纳姆轮底盘控制程序，包含速度分解 | Control Algorithm for chassises with Mecanum wheel, including velocity decompose
│   ├── chassis_logic.h
│   └── ...                               其他逻辑控制程序（近期更新） | Other high level logical control algorithms (Might Refactor Soon)
├── common                              - 通用组件 | Universal Components -
│   ├── common_macro.h
│   ├── CRC8.cpp
│   ├── CRC8.h
│   ├── CRC16.cpp
│   └── CRC16.h
├── debug                               - 调试相关代码 | Code for Tests -
│   ├── shell                             内嵌终端 | Integrated Shell -
│   │   ├── printf.c
│   │   ├── printf.h
│   │   ├── shell.cpp                     Shell 终端接口 | Shell Interface -
│   │   ├── shell.h
│   │   ├── shell_base.c
│   │   ├── shell_base.h
│   │   ├── shell_base_cmd.c
│   │   ├── shell_base_cmd.h
│   │   ├── shell_dbg_cmd.cpp             Shell 通用调试命令 | Universal Shell Commands -
│   │   ├── shell_dbg_cmd.h
│   │   └── shellconf.h                   Shell 配置文件 | Shell configuration -
│   ├── unit_tests                      - 单元测试 | Unit Tests-
│   └── param_adjusts                   - 参数整调程序 | Parameter Adjust Programs -
├── module                              - 模块（将来可能与common合并） | common modules (might merge with common soon) -                       
├── chconf.h                            ChibiOS 配置文件 | ChibiOS configuration
└── halconf.h                           ChibiOS HAL 配置文件 | ChibiOS HAL configuraion
```

# 前置阅读资料 | Pre-request
* 工具链相关资料，参考 [Meta-Infantry Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki)
* Toolchain, [Meta-Infantry Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki)
* [CMake 构建系统简介](https://github.com/Meta-Team/Meta-Infantry/wiki/CMake-%E6%9E%84%E5%BB%BA%E7%B3%BB%E7%BB%9F%E7%AE%80%E4%BB%8B)
* CMake[Brief Introduction to CMake](https://github.com/Meta-Team/Meta-Infantry/wiki/CMake-%E6%9E%84%E5%BB%BA%E7%B3%BB%E7%BB%9F%E7%AE%80%E4%BB%8B)
* [Meta Team C++ Style Guide](https://github.com/Meta-Team/Meta-Infantry/wiki/Meta-Team-C---Style-Guide)
