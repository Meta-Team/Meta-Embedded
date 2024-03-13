# PA INFANTRY步兵调参固件
此target是为步兵编写的调参固件。这个固件取消了电机之间的界限，将每个电机看作独立的轴，每个电机的PID参数都可单独调整。你甚至可以为步兵底盘电机调整角度PID，对底盘M3508电机进行角度闭环控制(虽然不知道有什么用，但这听上去很酷)。

## 程序结构
本程序基本沿用了infantry application内的文件， 包含 `hardware_conf.h`,`can_motor_config`,`thread_priorities.h`，以及`pa_infantry.cpp`。

### pa_infantry.cpp
`pa_infantry.cpp`是调参的主程序文件，包含了调参的各类shell commands以及remote controller测试程序。

在遥控器左上拨钮拨到上方时，所有电机关闭并失去动力。

在遥控器左上拨钮拨到中间时，整机进入调试状态，用户可以通过发送shell command调试各个电机的角度或者速度PID。

在遥控器左上拨钮拨到下方时，整机进入测试状态，用户可以通过遥控器验证PID参数效果。

pa_infantry包含的shell command API会在文尾列出。

### hardware_conf.h
`hardware_conf.h`通过配置文件内TRUE/FALSE开启、关闭特定功能。
目前，用户可以改变 `#define ENABLE_USB_SHELL` 宏定义切换Shell显示方式。

若`ENABLE_USB_SHELL`为`TRUE`，你可以通过USB虚拟串口接入Shell。但是，由于USB虚拟串口包含接收验证，在USB断开时，调用Shell发送数据的线程会被Shell printf命令卡住导致机器停止接收命令。

若`ENABLE_USB_SHELL`为`FALSE`, 你可以通过UART6口，连接USB转TTL模块接入shell。此种方式不会导致某些调用Shell printf命令的线程卡死。

不过，由于大部分线程已经对Shell printf进行了分离，不管是使用USB虚拟串口还是UART6口，断开都只会影响命令接收而不会影响主功能（电机控制PID，遥控器）卡死。

### thread_priorities.h
`thread_priorities.h`包含了线程的优先级，由于作者较懒和考虑到未来开发可能在调参程序中加入的功能，这个文件是直接从infantry拷贝过来的，有较多冗余的priorities。

### can_motor_config
`can_motor_config` 类决定了电机的数量、以及对应电机的型号和`SID`。此外，`can_motor_config`中还包含了一组电机的初始PID参数。在每次重启后，即使你不设置pid电机仍然是可以动起来的，这不是灵异现象。
关于详细的`can_motor_config`介绍，请阅读`dev/interface/can_motor`下的`README`文档。

## pa_infantry Shell 命令一览
| 命令头               |  数据   |          详细描述           |
|:------------------|:-----:|:-----------------------:|
| set_enable_a      | motor_id |   启动角度PID闭环控制（串级PID）    |
| set_disable_a     | motor_id  | 关闭角度PID闭环控制（并将力矩输出调整为0） |
| set_enable_v      | motor_id |       启动速度PID闭环控制       |
| set_disable_v     | motor_id  |       关闭速度PID闭环控制       |
| get_sid           | motor_id  |     得到某个电机的CAN SID      |
| fb_enable         | motor_id  |   将某个电机的反馈数据显示在Shell上   |
| fb_disable        | motor_id  |  停止把某个电机的反馈数据显示在Shell上  |
| set_pid           | motor_id pid_id(0: angle_to_v, 1: v_to_i) ki kp kd i_limit out_limit  |         设置PID参数         |
| echo_pid          | motor_id pid_id(0: angle_to_v, 1: v_to_i)  |      读取正在使用的PID参数       |
| set_target_angle  | motor_id target_angle  |        设置电机目标角度         |
| set_target_vel    | motor_id target_vel  |        设置电机目标速度         |
| echo_actual_angle | motor_id  |        反馈电机累计角度         |
| echo_raw_angle    | motor_id  | 反馈电机原始转子角度（-8192~8192）  |

关于Shell反馈数据格式，在CANMotorController的DOXYGEN documentation中有解释:
```
 !fb, %u, %u,  %.2f,   %.2f,  %.2f,    %.2f,    %d,     %d\r\n
       |   |    |       |      |        |        |       |
       |   |  actual target  actual   target   actual  target
     time id  angle  angle   velocity velocity current current
```
其中，`!fb`是行头，标志这行是反馈数据。

`time`为反馈数据的时间戳，从STM32开机开始计时，单位为毫秒。

`id`为电机ID，和`can_motor_config`中的`motor_id_t` enumerator相对应，**并非SID！**

其余的就不一一阐述了。

希望有兴趣的同学可以通过此写出自己的GUI调参软件 :D