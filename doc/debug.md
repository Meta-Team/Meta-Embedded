目前调试方法主要有两种——shell和dbg。

# Shell
本工程使用一个shell界面用于调试，shell线程的优先级为HIGHPRIO，以确保在其他线程阻塞时shell依然可用。

调试命令参考[shell.md](\shell.md).

# DBG
配置好OpenOCD和DBG后，可以进行外部调试。除了常用的步进、断点、查看变量外，还有以下操作可供使用。

### 查看Halt信息
当ChibiOS出现错误而进入Halt状态时，可以挂载Remote DBG后通过命令```p ch.dbg.panic_msg```查看halt的原因。

