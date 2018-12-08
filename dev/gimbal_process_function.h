//
// Created by admin on 2018/12/8.
//

#ifndef META_INFANTRY_GIMBAL_PROCESS_H
#define META_INFANTRY_GIMBAL_PROCESS_H

#include <stdio.h>
#include <stdint.h>

class gimbal_process {
public:
    uint8_t data8[8];
    typedef struct {
        uint16_t actual_angle_orig;
        int16_t actual_angle_last; //last actual angle
        int16_t actual_angle_base_round; // The number of round(s) that motor has rotated related to original position
        int16_t actual_angle;
        int16_t delta_angle;
        int16_t target_angle;
        int16_t target_current;
    } gimbal_motor;

    gimbal_motor motor[2];

    uint16_t gimbal_fi_orig[2];

    typedef struct {
        struct {
            uint8_t                 FMI;            /**< @brief Filter id.          */
            uint16_t                TIME;           /**< @brief Time stamp.         */
        };
        struct {
            uint8_t                 DLC : 4;          /**< @brief Data length.        */
            uint8_t                 RTR : 1;          /**< @brief Frame type.         */
            uint8_t                 IDE : 1;          /**< @brief Identifier type.    */
        };
        union {
            struct {
                uint32_t              SID : 11;         /**< @brief Standard identifier.*/
            };
            struct {
                uint32_t              EID : 29;         /**< @brief Extended identifier.*/
            };
        };
        union {
            uint8_t                 data8[8];       /**< @brief Frame data.         */
            uint16_t                data16[4];      /**< @brief Frame data.         */
            uint32_t                data32[2];      /**< @brief Frame data.         */
            uint64_t                data64[1];      /**< @brief Frame data.         */
        };
    } CANRxFrame;
    void process_gimbal_feedback(CANRxFrame *rxmsg);
};

#endif //META_INFANTRY_GIMBAL_PROCESS_H
