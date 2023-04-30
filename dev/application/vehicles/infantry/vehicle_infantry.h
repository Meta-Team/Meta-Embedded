//
// Created by liuzikai on 2019-04-22. Modified by ShaoXihe, May 2023
// This file contains common parameters for infantry
//

#ifndef META_INFANTRY_VEHICLE_INFANTRY_H
#define META_INFANTRY_VEHICLE_INFANTRY_H


/*===========================================================================*/
/*                            AHRS Configurations                            */
/*===========================================================================*/

#define MPU6500_BIAS_DATA_ID 0x0001
#define ON_BOARD_AHRS_MATRIX {{0.0f, -1.0f, 0.0f}, \
                              {1.0f, 0.0f, 0.0f}, \
                              {0.0f, 0.0f, 1.0f}}

#define GIMBAL_ANGLE_INSTALLATION_MATRIX {{1.0f, 0.0f, 0.0f}, \
                                          {0.0f, 1.0f, 0.0f}, \
                                          {0.0f, 0.0f, -1.0f}}
#define GIMBAL_GYRO_INSTALLATION_MATRIX {{0.0f,  1.0f, 0.0f}, \
                                         {0.0f,  0.0f,  -1.0f}, \
                                         {-1.0f, 0.0f,  0.0f}}


/*===========================================================================*/
/*               Gimbal and Shoot Installation Configurations                */
/*===========================================================================*/

#define GIMBAL_PITCH_MIN_ANGLE  (-20)  // up range for pitch [degree]
#define GIMBAL_PITCH_MAX_ANGLE  (5)   //  down range for pitch [degree]

#define SHOOT_DEGREE_PER_BULLET 45.0f  // rotation degree of bullet loader for each bullet


/*===========================================================================*/
/*                       Chassis Mechanism Parameters                        */
/*===========================================================================*/

#define CHASSIS_WHEEL_BASE  420.0f                     // distance between front axle and the back axle [mm]
#define CHASSIS_WHEEL_TREAD 372.0f                     // distance between left and right wheels [mm]
#define CHASSIS_WHEEL_CIRCUMFERENCE 478.0f             // circumference of wheels [mm]

#define CHASSIS_CLIP_PID_THETA2V_KP 10.0f
#define CHASSIS_CLIP_PID_THETA2V_KI 0.02f
#define CHASSIS_CLIP_PID_THETA2V_KD 0.0f
#define CHASSIS_CLIP_PID_THETA2V_I_LIMIT 60.0f
#define CHASSIS_CLIP_PID_THETA2V_OUT_LIMIT 270.0f
#define CHASSIS_CLIP_PID_THETA2V_PARAMS \
    {CHASSIS_CLIP_PID_THETA2V_KP, CHASSIS_CLIP_PID_THETA2V_KI, CHASSIS_CLIP_PID_THETA2V_KD, \
    CHASSIS_CLIP_PID_THETA2V_I_LIMIT, CHASSIS_CLIP_PID_THETA2V_OUT_LIMIT}

#define CHASSIS_FOLLOW_PID_THETA2V_KP 12.5f
#define CHASSIS_FOLLOW_PID_THETA2V_KI 0
#define CHASSIS_FOLLOW_PID_THETA2V_KD 20.0f
#define CHASSIS_FOLLOW_PID_THETA2V_I_LIMIT 0
#define CHASSIS_FOLLOW_PID_THETA2V_OUT_LIMIT 500.0f
#define CHASSIS_FOLLOW_PID_THETA2V_PARAMS \
    {CHASSIS_FOLLOW_PID_THETA2V_KP, CHASSIS_FOLLOW_PID_THETA2V_KI, CHASSIS_FOLLOW_PID_THETA2V_KD, \
    CHASSIS_FOLLOW_PID_THETA2V_I_LIMIT, CHASSIS_FOLLOW_PID_THETA2V_OUT_LIMIT}

#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KP 30.0f
#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KI 15.0f
#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KD 0.0f
#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_I_LIMIT 720.0f
#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_OUT_LIMIT 600.0f
#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_PARAMS \
    {CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KP, CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KI, CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KD,\
     CHASSIS_LOGIC_DODGE_OMEGA2VOLT_I_LIMIT, CHASSIS_LOGIC_DODGE_OMEGA2VOLT_OUT_LIMIT}


/*===========================================================================*/
/*                                  Vision                                   */
/*===========================================================================*/

#define VISION_BASIC_CONTROL_DELAY      100   /* ms */
#define VISION_DEFAULT_BULLET_SPEED     15.0f /* mm/ms = m/s */
#define VISION_SHOOT_TOLERANCE          7   /* ms */


/*===========================================================================*/
/*                         Dev Board LED Usage List                          */
/*===========================================================================*/

#define DEV_BOARD_LED_SYSTEM_INIT 1
#define DEV_BOARD_LED_CAN         2
#define DEV_BOARD_LED_AHRS        3
#define DEV_BOARD_LED_REMOTE      4
#define DEV_BOARD_LED_GIMBAL      5
#define DEV_BOARD_LED_CHASSIS     6
#define DEV_BOARD_LED_REFEREE     7  // used in infantry ShootLG BulletCounterThread
#define DEV_BOARD_LED_SD_CARD     8


/*===========================================================================*/
/*                          User Client Usage List                           */
/*===========================================================================*/

#define USER_CLIENT_FW_STATE_LIGHT                  0
#define USER_CLIENT_DODGE_MODE_LIGHT                1
#define USER_CLIENT_SUPER_CAPACITOR_STATUS_LIGHT    2
#define USER_CLIENT_SPEED_LEVEL_3_LIGHT             3
#define USER_CLIENT_SPEED_LEVEL_2_LIGHT             4
#define USER_CLIENT_SPEED_LEVEL_1_LIGHT             5

#define USER_CLIENT_FW_SPEED_NUM                    2
//#define USER_CLIENT_REMAINING_HEAT_NUM            2
#define USER_CLIENT_ACQUIRED_BULLET_NUM             3
//#define USER_CLIENT_ACTUAL_POWER_NUM              2
#define USER_CLIENT_SUPER_CAPACITOR_VOLTAGE_NUM     1

/*===========================================================================*/
/*                     Chassis Capacitor Configurations                      */
/*===========================================================================*/

#define SUPER_CAPACITOR_WARNING_VOLTAGE   15

#endif //META_INFANTRY_VEHICLE_INFANTRY_H
