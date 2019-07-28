//
// Created by liuzikai on 2019-01-28.
//

#ifndef META_INFANTRY_MAIN_INFANTRY_FIVE_H
#define META_INFANTRY_MAIN_INFANTRY_FIVE_H

#include "vehicle_infantry.h"

/** Installation Based Params **/

// Raw angle of yaw and pitch when gimbal points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts
#define GIMBAL_YAW_FRONT_ANGLE_RAW 3990
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 7256  // of no use now

#define MPU6500_STORED_GYRO_BIAS {0.956225752f, -0.640122234f, 0.300034373f}


#endif //META_INFANTRY_MAIN_INFANTRY_FIVE_H
