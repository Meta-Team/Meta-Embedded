//
// Created by liuzikai on 2019-01-30.
//

#ifndef META_INFANTRY_VEHICLE_ENGINEER_H
#define META_INFANTRY_VEHICLE_ENGINEER_H

/*** Installation Based Params ***/

// Raw angle of motor when robotic arm is placed inside.
//   Note: the program will echo the raw angles of the motor as the program starts
#define ROBOTIC_ARM_INSIDE_ANGLE_RAW 4206

/*** Chassis PID Params ***/

#define CHASSIS_PID_V2I_KP 8.0f
#define CHASSIS_PID_V2I_KI 0.0f
#define CHASSIS_PID_V2I_KD 0.0f
#define CHASSIS_PID_V2I_I_LIMIT 3000.0f
#define CHASSIS_PID_V2I_OUT_LIMIT 7000.0f
#define CHASSIS_PID_V2I_PARAMS \
    CHASSIS_PID_V2I_KP, CHASSIS_PID_V2I_KI, CHASSIS_PID_V2I_KD, \
    CHASSIS_PID_V2I_I_LIMIT, CHASSIS_PID_V2I_OUT_LIMIT

#endif //META_INFANTRY_VEHICLE_ENGINEER_H
