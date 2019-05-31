# Meta Team - Infantry 2019

ZJU-UIUC Meta 战队 RoboMaster 嵌入式程序工程，基于 ChibiOS 操作系统，使用 C 和 C++。

工程包含步兵、工程、英雄机器人源代码，适用于 RM 2017、RM 2018 A 型开发板，通过条件编译控制编译目标。

# 基本结构

更新日期：2019.5.12

```
Meta-Infantry
├── config           CMake 配置、OpenOCD 配置、辅助脚本等
├── dev              我们编写的源码文件
├── os               ChibiOS 源码文件
├── CMakeLists.txt   CMake 主配置文件
└── README.md        此文件
```

工程使用类实现封装和模块化，使用线程控制模块间的调用和信息传递。


```
├── board
│   ├── rm_board_2017
│   │   ├── board.c
│   │   ├── board.h
│   │   ├── board.mk
│   │   └── mcuconf.h
│   └── rm_board_2018_a
│       ├── board.c
│       ├── board.h
│       ├── board.mk
│       └── mcuconf.h
├── common                              - 通用组件 -
│   └── common_macro.h
├── control                             较高层次的控制器模块
│   ├── chassis.cpp                     Chassis 高层底盘控制模块
│   ├── chassis.h
│   ├── gimbal.cpp                      Gimbal 高层云台控制模块
│   ├── gimbal.h
│   ├── shoot.cpp                       Shoot 高层发射机构控制模块
│   ├── shoot.h
│   ├── state_handler.cpp               StateHandler 状态控制器
│   └── state_handler.h
├── debug                               - 调试相关代码 -
│   ├── unit_tests                      - 单元测试 -
│   ├── serial_shell.cpp                Shell 串口调试界面
│   ├── serial_shell.h
│   ├── shell_debug_commands.cpp        Shell 通用调试命令
│   ├── shell_debug_commands.h
│   └── shellconf.h                     Shell 配置文件
├── interface                           - 较低层次的接口模块 -
│   ├── attitude_calc.cpp
│   ├── attitude_calc.h
│   ├── buzzer.cpp                      Buzzer 蜂鸣器接口
│   ├── buzzer.h
│   ├── chassis_interface.cpp           ChassisInterface 低层底盘接口
│   ├── chassis_interface.h
│   ├── elevator_interface.cpp          ElevatorInterface 低层工程升降机构接口
│   ├── elevator_interface.h
│   ├── gimbal_interface.cpp            GimbalInterface 低层云台接口
│   ├── gimbal_interface.h
│   ├── led.cpp                         LED LED接口
│   ├── led.h
│   ├── mpu6500.cpp                     MPU6500 陀螺仪接口
│   ├── mpu6500.h
│   ├── mpu6500_reg.h
│   ├── remote_interpreter.cpp          Remote 遥控器接口
│   ├── remote_interpreter.h
│   ├── robotic_arm.cpp                 RoboticArm 低层工程机械臂接口
│   ├── robotic_arm.h
│   ├── sentry_chassis_interface.cpp    SentryChassisInterface 低层哨兵底盘接口
│   └── sentry_chassis_interface.h
├── module                              - 通用模块 -
│   ├── button_monitor.cpp
│   ├── button_monitor.h
│   ├── can_interface.cpp               CANInterface 底层CAN收发控制器
│   ├── can_interface.h
│   └── pid_controller.hpp              PIDController PID控制器
├── vehicle                             - 机器人主控线程代码 -
│   ├── engineer                        - 工程机器人 -
│   │   ├── elevator_thread.cpp
│   │   ├── elevator_thread.h
│   │   ├── main_engineer.cpp
│   │   ├── robotic_arm_thread.cpp
│   │   ├── robotic_arm_thread.h
│   │   └── vehicle_engineer.h
│   └── infantry                        - 步兵机器人 -
│       ├── main_infantry.cpp           模块初始化、启动自检、线程管理
│       ├── thread_chassis.hpp          底盘控制线程
│       ├── thread_error_detect.hpp     运行时错误检测模块
│       ├── thread_gimbal.hpp           云台控制线程
│       ├── thread_shoot.hpp            发射机构控制线程
│       ├── vehicle_infantry.h          步兵机器人通用配置信息
│       ├── vehicle_infantry_five.h     5号步兵配置信息
│       ├── vehicle_infantry_four.h     4号步兵配置信息
│       └── vehicle_infantry_three.h    3号步兵配置信息
├── chconf.h                            ChibiOS 配置文件
└── halconf.h                           ChibiOS HAL 配置文件


```

# 前置阅读资料
* 工具链相关资料，参考 [Meta-Infantry Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki)
* [CMake 构建系统简介](https://github.com/Meta-Team/Meta-Infantry/wiki/CMake-%E6%9E%84%E5%BB%BA%E7%B3%BB%E7%BB%9F%E7%AE%80%E4%BB%8B)
* [Meta Team C++ Style Guide](https://github.com/Meta-Team/Meta-Infantry/wiki/Meta-Team-C---Style-Guide)