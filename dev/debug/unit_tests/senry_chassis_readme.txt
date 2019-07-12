由于senrty_chassis_matlab_unit_test.cpp是基于对hero_shoot_param_adjust.cpp的简单模仿，在matlab上用的是对云台的调试文件，
功能相对简单，无法单独在matlab上实现所有指令，部分指令需手动在终端上输入。以下是所有指令概览：

        {"g_enable",   cmd_chassis_enable},
        {"g_enable_fb", cmd_chassis_enable_feedback},
        {"g_set_v",     cmd_set_target_velocities},
        {"c_set_mode",    cmd_chassis_set_mode},
        {"g_set_params",  cmd_chassis_set_pid},
        {"g_echo_params",   cmd_chassis_print_pid},
        {"g_set_angle",   cmd_chassis_set_position},
        {"g_fix", cmd_chassis_clear_position},
        {"c_pos", cmd_chassis_print_position},
        {"c_cur", cmd_chassis_print_current},
        {"c_v", cmd_chassis_print_velocity},
        {"c_testc", cmd_chassis_test_current}

其中“g”开头的指令是可以通过matlab端操作的指令，“c”开头的指令需要手动在终端输入

底盘初始状态说明：
    电机——disable
    运动模式——STOP_MODE
    当前位置——0 (cm)，可能会有一些小数误差，启动前需要位置归零
    当前目标位置——0 (cm)
    当前目标速度——0 (cm/s)
    最大速度——110 (cm/s)
    自动模式半径——30 (cm)
    恒流模式电流——1000 (mA)
    变速模式——false
    反馈线程——false
    PID参数——均为0

指令调试流程的说明：

Step 1: MATLAB设置PID参数 ("g_set_params")

Step 2: MATLAB清空位置 ("g_fix")

Step 3: 终端输入运动模式 ("c_set_mode")
    DEFAULT STOP_MODE     发送0电流
    1 ONE_STEP_MODE       一个指令运动一段距离后停下
    2 SHUTTLED_MODE       自动模式，哨兵以当前位置为原点在[-radius, radius]的范围内往复运动
    3 V_MODE
    4 FINAL_AUTO_MODE

Step 4: MATLAB启动电机 ("g_enable")
    可能两个电机分别有启动选择，但只要其中一个选择启动则两个电机均启动

Step 5：MATLAB启动反馈 ("g_enable_fb")

Step 6.1: MATLAB改变电机速度 ("g_set_v")
    界面会显示设置两个电机的速度，但实际上两个电机的最大速度应该是一样的，所以两个速度设置成一样的

Step 6.2: MATLAB查看PID参数 ("g_echo_params")

Step 6.3: MATLAB设置单步位移 ("g_set_angle")
    在ONE_STEP_MODE状态下使用
    界面会显示设置两个电机的“角度”，但实际上两个电机的位移应该是一样的，所以两个位移设置成一样的，单位：cm

Step 6.4: 单独查看电流，速度，位置 ("c_cur", "c_v", "c_pos")

Step 6.5: 设置恒流模式 ("c_testc")
    发送恒定正电流1000mA，操作时哨兵必须脱离轨道！
    命令输入一次开启，再输入一次关闭，如此往复
    启动时将无视电机反馈，无反馈调节，此指令目的在于确认电流信号发送正常以及判断电机转动方法与正目标电流的关系


关于全自动模式的底盘运动说明：

    1. 开启全自动模式时，哨兵左侧必须处于距离轨道左端 20cm 处，以便符合代码中设计的位置节点
    2. 开启全自动模式后，哨兵以 巡航速度 自动移至直道段右端，并开始在直道段来回移动
    3. 若检测到血量下降（被击中），则
        (1) 哨兵处于直道段，则顺着目前运动方向以 躲避速度 移至对应的弯道段进行躲避
        (2) 哨兵处于弯道段，则立即以 躲避速度 前往另一个弯道段进行躲避
    4. 到达弯道躲避段的端点后恢复 巡航速度 并开始计时，若 20 秒内未检查到血量下降（未被击中），则以 巡航速度 移动回直道段
