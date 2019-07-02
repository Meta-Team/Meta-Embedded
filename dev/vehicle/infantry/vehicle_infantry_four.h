//
// Created by liuzikai on 2019-01-28.
//

#ifndef META_INFANTRY_MAIN_INFANTRY_FOUR_H
#define META_INFANTRY_MAIN_INFANTRY_FOUR_H

#include "vehicle_infantry.h"

// Raw angle of yaw and pitch when gimbal points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts
#define GIMBAL_YAW_FRONT_ANGLE_RAW 4344
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 2866

#define MPU6500_STORED_GYRO_BIAS {0.682773649f, -0.682926177f, -0.257317185f}

#endif //META_INFANTRY_MAIN_INFANTRY_FOUR_H
