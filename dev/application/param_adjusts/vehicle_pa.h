//
// Created by liuzikai on 2019-04-22.
// Edited by Qian Chen & Mo Kanya & Jing Tenjun on 2019-07-05
// This file contains common parameters for hero
//

#ifndef META_INFANTRY_VEHICLE_INFANTRY_H
#define META_INFANTRY_VEHICLE_INFANTRY_H


/// AHRS Configurations
#define ON_BOARD_AHRS_MATRIX {{0.0f, -1.0f, 0.0f}, \
                              {1.0f, 0.0f, 0.0f}, \
                              {0.0f, 0.0f, 1.0f}}
// Raw angle of yaw and pitch when gimbal points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts

#define GIMBAL_YAW_FRONT_ANGLE_RAW 4818
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 4450
#define GIMBAL_SUB_PITCH_FRONT_ANGLE_RAW 3569


#define LOADER_SHOOT_DEGREE_PER_BULLET 72.0f

#define MPU6500_BIAS_DATA_ID 0x0001
#define MPU6500_STORED_GYRO_BIAS {0.488913863, -0.318009287, 0.596762299}
#define GIMBAL_ANGLE_INSTALLATION_MATRIX {{1.0f, 0.0f, 0.0f}, \
                                          {0.0f, 1.0f, 0.0f}, \
                                          {0.0f, 0.0f, -1.0f}}


#define GIMBAL_GYRO_INSTALLATION_MATRIX {{0.0f,  1.0f, 0.0f}, \
                                         {0.0f,  0.0f,  1.0f}, \
                                         {1.0f, 0.0f,  0.0f}}

/// Chassis Mechanism Parameters
#define CHASSIS_WHEEL_BASE  550.0f                // distance between front axle and the back axle [mm]
#define CHASSIS_WHEEL_TREAD 500.0f                    // distance between left and right wheels [mm]
#define CHASSIS_WHEEL_CIRCUMFERENCE 478.0f             // circumference of wheels [mm]
#define CHASSIS_GIMBAL_OFFSET 0.0f                    // distance between center of gimbal and the center of chassis

/// Dev Board LED Usage List
#define DEV_BOARD_LED_SYSTEM_INIT 1
#define DEV_BOARD_LED_CAN         2
#define DEV_BOARD_LED_AHRS        3
#define DEV_BOARD_LED_REMOTE      4
#define DEV_BOARD_LED_GIMBAL      5
#define DEV_BOARD_LED_CHASSIS     6
#define DEV_BOARD_LED_REFEREE     7
#define DEV_BOARD_LED_SD_CARD     8

/// Super Capacitor Configurations
#define SUPER_CAPACITOR_WARNING_VOLTAGE   15

#endif //META_INFANTRY_VEHICLE_INFANTRY_H
