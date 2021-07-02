# Meta Team - Infantry 2021

=> 请务必阅读 [Project Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki) :smiley:

ZJU-UIUC Meta 战队 RoboMaster 嵌入式程序工程，基于 ChibiOS 操作系统，使用 C 和 C++。

工程包含步兵、工程、英雄、哨兵机器人源代码，适用于 RM 2018 A 型、RM 2017 开发板，通过条件编译控制编译目标。

# 基本结构

更新日期：2019.5.31

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
.
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
├── control                             - 较高层次的控制器模块 -
│   ├── chassis.cpp                     Chassis 高层底盘（步兵/英雄/工程）控制模块
│   ├── chassis.h
│   ├── elevator.cpp                    Elevator 高层升降机构控制模块
│   ├── elevator.h
│   ├── gimbal.cpp                      Gimbal 高层云台（步兵/英雄）控制模块
│   ├── gimbal.h
│   ├── sentry_chassis.cpp              SentryChassisSKD 高层底盘（哨兵）控制模块
│   ├── sentry_chassis.h
│   ├── shoot.cpp                       Shoot 高层发射机构控制模块
│   ├── shoot.h
│   ├── state_handler.cpp               StateHandler 状态控制器
│   └── state_handler.h
├── debug                               - 调试相关代码 -
│   ├── shell                           - 内嵌终端 -
│   │   ├── printf.c
│   │   ├── printf.h
│   │   ├── shell.cpp                   Shell 终端接口
│   │   ├── shell.h
│   │   ├── shell_base.c
│   │   ├── shell_base.h
│   │   ├── shell_base_cmd.c
│   │   ├── shell_base_cmd.h
│   │   ├── shell_dbg_cmd.cpp           Shell 通用调试命令
│   │   ├── shell_dbg_cmd.h
│   │   └── shellconf.h                 Shell 配置文件
│   └── unit_tests                      - 单元测试 -
├── interface                           - 较低层次的接口模块 -
│   ├── imu                             - IMU 模块 -
│   │   ├── ahrs.cpp                    综合运算模块
│   │   ├── ahrs.h
│   │   ├── imu_math.hpp
│   │   ├── ist8310.cpp                 IST8310 接口
│   │   ├── ist8310.h
│   │   ├── ist8310_reg.h
│   │   ├── mpu6500.cpp                 MPU6500 陀螺仪接口
│   │   ├── mpu6500.h
│   │   └── mpu6500_reg.h
│   ├── buzzer.cpp                      BuzzerSKD 蜂鸣器接口
│   ├── buzzer.h
│   ├── chassis_interface.cpp           ChassisInterface 低层底盘接口
│   ├── chassis_interface.h
│   ├── elevator_interface.cpp          ElevatorInterface 低层工程升降机构接口
│   ├── elevator_interface.h
│   ├── gimbal_interface.cpp            GimbalInterface 低层云台/发射机构接口
│   ├── gimbal_interface.h
│   ├── led.cpp                         LED LED接口
│   ├── led.h
│   ├── referee_interface.cpp           Referee 裁判系统接口
│   ├── referee_interface.h
│   ├── remote_interpreter.cpp          Remote 遥控器接口
│   ├── remote_interpreter.h
│   ├── robotic_arm.cpp                 RoboticArm 低层工程机械臂接口
│   ├── robotic_arm.h
│   ├── sentry_chassis_interface.cpp    SentryChassisIF 哨兵低层底盘接口
│   └── sentry_chassis_interface.h
├── skeduler                            - 基于Interface的底层调度线程-
│   ├── chassis_skeduler.cpp
│   ├── chassis_skeduler.h
│   ├── engineer_chassis_skd.cpp
│   ├── engineer_chassis_skd.h
│   ├── engineer_elevator_skd.cpp
│   ├── engineer_elevator_skd.h
│   ├── gimbal_skeduler.cpp
│   ├── gimbal_skeduler.h
│   ├── robotic_arm_skd.cpp
│   ├── robotic_arm_skd.h
│   ├── sentry_chassis_skeduler.cpp
│   ├── sentry_chassis_skeduler.h
│   ├── shoot_skeduler.cpp
│   ├── shoot_skeduler.h
├── module                              - 通用模块 -
│   ├── CRC16.cpp                       CRC16 校验代码
│   ├── CRC16.h
│   ├── CRC8.cpp                        CRC8 校验代码
│   ├── CRC8.h
│   ├── button_monitor.cpp
│   ├── button_monitor.h
│   ├── can_interface.cpp               CANInterface 底层CAN收发控制器
│   ├── can_interface.h
│   └── pid_controller.hpp              PIDController PID控制器
├── vehicle                             - 机器人主控线程代码 -
│   ├── engineer                        - 工程机器人 -
│   │   ├── main_engineer.cpp
│   │   ├── state_machine_bullet_fetch.cpp
│   │   ├── state_machine_bullet_fetch.h
│   │   ├── state_machine_stage_climb.cpp
│   │   ├── state_machine_stage_climb.h
│   │   ├── thread_action_trigger.hpp
│   │   ├── thread_chassis.cpp
│   │   ├── thread_chassis.h
│   │   ├── thread_elevator.cpp
│   │   ├── thread_elevator.h
│   │   ├── thread_error_detect.hpp
│   │   └── vehicle_engineer.h
│   ├── hero                            - 英雄机器人 -
│   │   ├── main_hero.cpp
│   │   ├── thread_chassis.hpp
│   │   ├── thread_error_detect.hpp
│   │   ├── thread_gimbal.hpp
│   │   ├── thread_shoot.hpp
│   │   └── vehicle_hero.h
│   └── infantry                        - 步兵机器人 -
│       ├── main_infantry.cpp
│       ├── thread_chassis.hpp
│       ├── thread_error_detect.hpp
│       ├── thread_gimbal.hpp
│       ├── thread_shoot.hpp
│       ├── vehicle_infantry.h
│       ├── vehicle_infantry_five.h
│       ├── vehicle_infantry_four.h
│       └── vehicle_infantry_three.h
├── chconf.h                            ChibiOS 配置文件
└── halconf.h                           ChibiOS HAL 配置文件
```

# 前置阅读资料
* 工具链相关资料，参考 [Meta-Infantry Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki)
* [CMake 构建系统简介](https://github.com/Meta-Team/Meta-Infantry/wiki/CMake-%E6%9E%84%E5%BB%BA%E7%B3%BB%E7%BB%9F%E7%AE%80%E4%BB%8B)
* [Meta Team C++ Style Guide](https://github.com/Meta-Team/Meta-Infantry/wiki/Meta-Team-C---Style-Guide)
