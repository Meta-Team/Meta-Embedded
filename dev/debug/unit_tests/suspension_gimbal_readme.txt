懸掛式雲台測試文件說明：

初始狀態說明：
    shoot_mode——OFF
    PID參數——全為0
    當前實際角度——(0, 0)
    前角度——(0, 0)
    目標角度——(0, 0)
    YAW最大角度——170 degree
    PITCH最大角度——30 degree
    撥彈電機轉速——40 degree/s
    連發模式——false
    摩擦輪佔空比——{0.0, 0.1, 0.3}
    是否允許a2v——false
    YAW是否固定——false

指令概覽：
        {"g_enable",                cmd_gimbal_enable},
        {"g_enable_fb",             cmd_gimbal_enable_feedback},
        {"g_front",                 cmd_gimbal_set_front_angle},
        {"g_set_v",                 cmd_gimbal_set_target_velocities},
        {"g_set_angle",             cmd_gimbal_set_target_angle},
        {"g_set_params",            cmd_gimbal_set_parameters},
        {"g_echo_params",           cmd_gimbal_echo_parameters},
        {"g_enable_fw",             cmd_gimbal_enable_fw},
        {"g_continuous_shoot",      cmd_gimbal_continuous_shooting},
        {"g_incontinuous_shoot",    cmd_gimbal_incontinuous_shooting},
        {"g_check",                 cmd_gimbal_check},
        {"g_fix",                   cmd_fix_front},
