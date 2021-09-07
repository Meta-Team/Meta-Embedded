//
// Created by liuzikai on 2019-01-28.
//

#ifndef META_INFANTRY_MAIN_INFANTRY_FIVE_H
#define META_INFANTRY_MAIN_INFANTRY_FIVE_H

#include "vehicle_infantry.h"

/** Installation Based Params **/

// Raw angle of yaw and pitch when gimbal points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts
#define GIMBAL_YAW_FRONT_ANGLE_RAW 2024
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 7256  // of no use now

#define MPU6500_STORED_GYRO_BIAS {0.491007685, -1.024637818, -0.371067702}

#define SHOOT_FW_SPEED 2750

#endif //META_INFANTRY_MAIN_INFANTRY_FIVE_H
