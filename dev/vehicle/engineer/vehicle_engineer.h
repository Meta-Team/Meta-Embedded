//
// Created by liuzikai on 2019-01-30.
//

#ifndef META_INFANTRY_VEHICLE_ENGINEER_H
#define META_INFANTRY_VEHICLE_ENGINEER_H

/*** Installation Based Params ***/

// Raw angle of motor when robotic arm is placed inside.
//   Note: the program will echo the raw angles of the motor as the program starts
#define ROBOTIC_ARM_INSIDE_ANGLE_RAW 4206

/** Mechanism Parameters */
// TODO: to be determine
#define CHASSIS_WHEEL_BASE  0.0// distance between front axle and the back axle, mm
#define CHASSIS_WHEEL_TREAD  0.0// distance between left and right wheels, mm
#define CHASSIS_WHEEL_CIRCUMFERENCE  0.0// mm

/*** Chassis PID Params ***/

#define CHASSIS_PID_V2I_KP 8.0f
#define CHASSIS_PID_V2I_KI 0.0f
#define CHASSIS_PID_V2I_KD 0.0f
#define CHASSIS_PID_V2I_I_LIMIT 3000.0f
#define CHASSIS_PID_V2I_OUT_LIMIT 7000.0f
#define CHASSIS_PID_V2I_PARAMS \
    CHASSIS_PID_V2I_KP, CHASSIS_PID_V2I_KI, CHASSIS_PID_V2I_KD, \
    CHASSIS_PID_V2I_I_LIMIT, CHASSIS_PID_V2I_OUT_LIMIT

// TODO: to be determine
#define ELEVATOR_PID_A2V_KP 8.0f
#define ELEVATOR_PID_A2V_KI 0.0f
#define ELEVATOR_PID_A2V_KD 0.0f
#define ELEVATOR_PID_A2V_V_LIMIT 3000.0f
#define ELEVATOR_PID_A2V_OUT_LIMIT 7000.0f
#define ELEVATOR_PID_A2V_PARAMS \
    ELEVATOR_PID_A2V_KP, ELEVATOR_PID_A2V_KI, ELEVATOR_PID_A2V_KD, \
    ELEVATOR_PID_A2V_V_LIMIT, ELEVATOR_PID_A2V_OUT_LIMIT

#define ELEVATOR_PID_V2I_KP 8.0f
#define ELEVATOR_PID_V2I_KI 0.0f
#define ELEVATOR_PID_V2I_KD 0.0f
#define ELEVATOR_PID_V2I_I_LIMIT 3000.0f
#define ELEVATOR_PID_V2I_OUT_LIMIT 7000.0f
#define ELEVATOR_PID_V2I_PARAMS \
    ELEVATOR_PID_V2I_KP, ELEVATOR_PID_V2I_KI, ELEVATOR_PID_V2I_KD, \
    ELEVATOR_PID_V2I_I_LIMIT, ELEVATOR_PID_V2I_OUT_LIMIT

#endif //META_INFANTRY_VEHICLE_ENGINEER_H
