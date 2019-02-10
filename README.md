# Meta Team - Infantry 2018

This is the repository of infantry. It's based on RM dev board 2017.

## 开发板适配（board.h)

### RM 2017 开发板

- [x] LED GPIO
- [ ] 用户自定义按键
- [ ] 蜂鸣器

### RM 2018 A型 开发板

- [x] LED GPIO
- [ ] 用户自定义按键
- [x] 蜂鸣器

## Changes to ChibiOS

- [x] TIM12 PWM

## Repo Structure

* config: configurations for CMake and OpenOCD. <br>
* dev: Development files. <br>
    * board: Sources and headers for dev board. <br>
    * common: Common modules. <br>
    * debug: modules for debug. <br>
    * interfaces: modules for low-level communication.
* doc: Documents.
    * resource: images and other resources for doc.
* os: sources from ChibiOS. No need to read them.

## Toolchain Setup

See documents in ONES wiki.

## Add files and folders

This project uses Make as main building system. CMake is simply a
wrapper to make CLion work properly.

### When add a new source file:
#### 1. Add to corresponding location in **dev/dev.mk**

For example, if the new file names `test.cpp`, and belongs to
`DEV_MAIN_CPPSRC` catalog, append it to `DEV_MAIN_CPPSRC`.

For example, if the original content is:
```
DEV_MAIN_CPPSRC = interfaces/remote_interpreter.cpp \
                  interfaces/gimbal_process_function.cpp \
                  interfaces/send_currents_functions.cpp \
                  main.cpp
```
After appended it should become:
```
DEV_MAIN_CPPSRC = interfaces/remote_interpreter.cpp \
                  interfaces/gimbal_process_function.cpp \
                  interfaces/send_currents_functions.cpp \
                  main.cpp \
                  test.cpp
```

See comments in `dev.mk` to see what each catalog does.

#### 2. Add to CMakeLists.txt

This step is to allow CLion to recognize this file. If not, some warning
like this will appear:

![](./resource/README.png)

For example, if the original content is:
```
set(DEVCPPSRC
        dev/common/port_to_string.cpp
        dev/debug/button_monitor.cpp
        dev/debug/serial_shell.cpp
        dev/debug/serial_shell_commands.cpp
        dev/main.cpp)
```
After appended it should become:
```
set(DEVCPPSRC
        dev/common/port_to_string.cpp
        dev/debug/button_monitor.cpp
        dev/debug/serial_shell.cpp
        dev/debug/serial_shell_commands.cpp
        dev/main.cpp
        dev/test.cpp)
```

### When add a new directory

#### 1. Add to `DEV_COMMON_INC`, `DEV_MAIN_INC` or other section in **dev/dev.mk**

#### 2. Add to `include_directories` in CMakeLists.txt

## Coding and Styles Standards

See ONES wiki.