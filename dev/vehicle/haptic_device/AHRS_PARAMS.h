//
// Created by 钱晨 on 10/29/21.
//

#ifndef META_INFANTRY_AHRS_PARAMS_H
#define META_INFANTRY_AHRS_PARAMS_H

#define GIMBAL_ANGLE_INSTALLATION_MATRIX {{1.0f, 0.0f, 0.0f}, \
                                          {0.0f, 1.0f, 0.0f}, \
                                          {0.0f, 0.0f, -1.0f}}


#define GIMBAL_GYRO_INSTALLATION_MATRIX {{0.0f,  1.0f, 0.0f}, \
                                         {0.0f,  0.0f,  -1.0f}, \
                                         {-1.0f, 0.0f,  0.0f}}

#define ON_BOARD_AHRS_MATRIX {{0.0f, -1.0f, 0.0f}, \
                              {1.0f, 0.0f, 0.0f}, \
                              {0.0f, 0.0f, 1.0f}}

#define MPU6500_STORED_GYRO_BIAS {-1.006189346, 0.034967087, 0.724042654}

#endif //META_INFANTRY_AHRS_PARAMS_H
