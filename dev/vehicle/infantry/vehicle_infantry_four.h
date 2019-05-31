//
// Created by liuzikai on 2019-01-28.
//

#ifndef META_INFANTRY_MAIN_INFANTRY_FOUR_H
#define META_INFANTRY_MAIN_INFANTRY_FOUR_H

#include "vehicle_infantry.h"

/** Installation Based Params **/

// Raw angle of yaw and pitch when gimbal points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts
#define GIMBAL_YAW_FRONT_ANGLE_RAW 620
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 5684

// Relationship between MPU6500 data and gimbal angular velocities
#define GIMBAL_YAW_ACTUAL_VELOCITY (-MPU6500::angle_speed.y)
#define GIMBAL_PITCH_ACTUAL_VELOCITY (-MPU6500::angle_speed.x)

#endif //META_INFANTRY_MAIN_INFANTRY_FOUR_H
