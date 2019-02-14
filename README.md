# Meta Team - Infantry 2018

这是 ZJU-UIUC Meta 战队 RoboMaster 嵌入式程序工程，基于 ChibiOS 操作系统，使用 C 和 C++。

工程包含步兵、工程、英雄机器人源代码，适用于 RM 2017、RM 2018 A 型开发板，通过条件编译控制编译目标。

# 基本结构

```
Meta-Infantry
├── config           CMake 配置、OpenOCD 配置、辅助脚本等
├── dev              我们编写的源码文件
├── os               ChibiOS 源码文件
├── CMakeLists.txt   CMake 主配置文件
└── README.md        此文件
```

**工程使用类实现封装和模块化，主线程（main）负责各个模块间的调用和信息传递。** 而单元测试亦是通过替换不同的 main 实现。

有两类主要的类：
* Interface：接口，主要负责与底层的交互。例如云台接口 GimbalInterface，负责解析云台电机回传信息，转换为可用的角度，并且负责目标电流的发送。
* Controller：控制器：主要负责逻辑运算部分。例如云台运算器 GimbalController，使用来自 GimbalInterface 的角度回传、MPU6500 的角速度回传，运算得到目标电流，再传给 GimbalInterface。数据传递由主线程完成。

对于一些较为简单的机器人模块，例如扬声器，可能只有单个模块，可能被归类到 Interface 或 Controller。

```
dev
├── board            开发板定义相关文件
├── vehicle_configs  机器人定义头文件，包含机器人轴距、开发板放置方向等信息
├── common           通用组件，多为一些简单但常用的代码片段
├── module           通用模块，例如 PID 运算模块
├── control          Controllers
├── interfaces       Interfaces
├── debug            Debug 相关代码
├── main.cpp         主线程
├── chconf.h         ChibiOS 配置文件
├── halconf.h        ChibiOS HAL 配置文件
└── shellconf.h      ChibiOS Shell 配置文件

```

# 前置阅读资料
* 工具链相关资料，参考 [Meta-Infantry Wiki](https://github.com/Meta-Team/Meta-Infantry/wiki)
* [CMake 构建系统简介](https://github.com/Meta-Team/Meta-Infantry/wiki/CMake-%E6%9E%84%E5%BB%BA%E7%B3%BB%E7%BB%9F%E7%AE%80%E4%BB%8B)
* [Meta Team C++ Style Guide](https://github.com/Meta-Team/Meta-Infantry/wiki/Meta-Team-C---Style-Guide)


# Targets 编译目标

"ut_" 代表 unit test。

## INFANTRY_ONE
步兵 #1。
- [ ] RM Board 2017
- [ ] RM Board 2018 A

## ENGINEER
工程。
- [ ] RM Board 2017
- [ ] RM Board 2018 A

## ut_blink
闪灯程序。按下用户自定义按键会改变闪灯状态。
- [x] RM Board 2017
- [x] RM Board 2018 A

## ut_remote_interpreter
遥控器单元测试程序。使用 Shell 指令 `p` 反馈遥控器状态。
- [x] RM Board 2017
- [ ] RM Board 2018 A

## ut_gimbal_interface
GimbalInterface 单元测试程序。通过 Shell 回传云台电机信息、控制目标电流。
- [x] RM Board 2017
- [ ] RM Board 2018 A

## ut_gimbal
云台单元测试程序。通过 Shell 回传云台电机、PID信息、控制目标角度、角速度。配合 Meta-Terminal - Gimbal 调参工具使用。
- [x] RM Board 2017
- [ ] RM Board 2018 A

## ut_mpu6500
板载陀螺仪 MPU6500 单元测试。通过 Shell 回传开发板姿态信息。
- [x] RM Board 2017
- [ ] RM Board 2018 A

## ut_elevator
工程车升降机构单元测试。通过 Shell 设置目标高度。需要先配置好 RMDS 控制器
- [x] RM Board 2017
- [ ] RM Board 2018 A

## ut_buzzer
蜂鸣器单元测试程序。使用 Shell 指令 'buzzer ?' 播放声音。
- [x] RM Board 2017
- [x] RM Board 2018 A