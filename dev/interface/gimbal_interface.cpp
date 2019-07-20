//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback.
// Feng Chuhao wrote code about sending gimbal currents.
//

/**
 * @file    gimbal_interface.cpp
 * @brief   Interface to interact with low level driver of gimbal, including processing chassis motor feedback and
 *          sending target currents.
 *
 * @addtogroup gimbal
 * @{
 */

#include "gimbal_interface.h"
#include "common_macro.h"

GimbalIF::motor_feedback_t GimbalIF::feedback[MOTOR_COUNT];
int GimbalIF::target_current[MOTOR_COUNT] = {0, 0, 0};
float GimbalIF::fw_duty_cycle = 0.0f;
CANInterface *GimbalIF::can_ = nullptr;

const PWMConfig FRICTION_WHEELS_PWM_CFG = {
        50000,   // frequency
        1000,    // period
        nullptr, // callback
        {
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH0
                {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH1
                {PWM_OUTPUT_DISABLED, nullptr},    // CH2
                {PWM_OUTPUT_DISABLED, nullptr}     // CH3
        },
        0,
        0
};

void GimbalIF::init(CANInterface *can_interface, uint16_t yaw_front_angle_raw, uint16_t pitch_front_angle_raw,
        motor_type_t yaw_type, motor_type_t pitch_type, motor_type_t bullet_type, motor_type_t plate_type) {

    feedback[YAW].id = YAW;
    feedback[YAW].type = yaw_type;
    feedback[YAW].last_angle_raw = yaw_front_angle_raw;

    feedback[PITCH].id = PITCH;
    feedback[PITCH].type = pitch_type;
    feedback[PITCH].last_angle_raw = pitch_front_angle_raw;

    feedback[BULLET].id = BULLET;
    feedback[BULLET].type = bullet_type;
    feedback[BULLET].reset_front_angle();

    feedback[PLATE].id = PLATE;
    feedback[PLATE].type = plate_type;
    feedback[PLATE].reset_front_angle();

    can_ = can_interface;
    can_->register_callback(0x205, 0x208, process_motor_feedback);

    // Enable PWM and perform initialization on friction wheels

    pwmStart(&PWMD8, &FRICTION_WHEELS_PWM_CFG);

    pwmEnableChannel(&PWMD8, FW_LEFT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1 * 500 + 500));
    pwmEnableChannel(&PWMD8, FW_RIGHT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1 * 500 + 500));
    chThdSleep(TIME_MS2I(500));
    pwmEnableChannel(&PWMD8, FW_LEFT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0 * 500 + 500));
    pwmEnableChannel(&PWMD8, FW_RIGHT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0 * 500 + 500));

}

void GimbalIF::send_gimbal_currents() {

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x1FF;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    // Fill the current of Yaw
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[YAW], GIMBAL_INTERFACE_MAX_CURRENT);
#endif

if (feedback[YAW].type != RM6623) {
    txmsg.data8[0] = (uint8_t) (target_current[YAW] >> 8); // upper byte
    txmsg.data8[1] = (uint8_t) target_current[YAW];        // lower byte
} else {
    /**
     * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
     *       direction. In order to unified coordinate system, minus sign is applied here.
     */
    txmsg.data8[0] = (uint8_t) (-target_current[YAW] >> 8); // upper byte
    txmsg.data8[1] = (uint8_t) -target_current[YAW];        // lower byte
}

    // Fill the current of Pitch
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[PITCH], GIMBAL_INTERFACE_MAX_CURRENT);
#endif
    if (feedback[PITCH].type != RM6623) {
        txmsg.data8[2] = (uint8_t) (target_current[PITCH] >> 8); // upper byte
        txmsg.data8[3] = (uint8_t) target_current[PITCH];        // lower byte
    } else {
        /**
         * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
         *       direction. In order to unified coordinate system, minus sign is applied here.
         */
        txmsg.data8[2] = (uint8_t) (-target_current[PITCH] >> 8); // upper byte
        txmsg.data8[3] = (uint8_t) -target_current[PITCH];        // lower byte
    }

    // Fill the current of bullet loader
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[BULLET], GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT);
#endif
    if (feedback[BULLET].type != RM6623) {
        txmsg.data8[4] = (uint8_t) (target_current[BULLET] >> 8); // upper byte
        txmsg.data8[5] = (uint8_t) target_current[BULLET];       // lower byte
    } else {
        /**
         * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
         *       direction. In order to unified coordinate system, minus sign is applied here.
         */
        txmsg.data8[4] = (uint8_t) (-target_current[BULLET] >> 8); // upper byte
        txmsg.data8[5] = (uint8_t) -target_current[BULLET];       // lower byte
    }

    // Fill the current of plate
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[PLATE], GIMBAL_INTERFACE_BULLET_PLATE_MAX_CURRENT);
#endif
    if (feedback[PLATE].type != RM6623) {
        txmsg.data8[6] = (uint8_t) (target_current[PLATE] >> 8);
        txmsg.data8[7] = (uint8_t) target_current[PLATE];
    } else {
        /**
         * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
         *       direction. In order to unified coordinate system, minus sign is applied here.
         */
        txmsg.data8[6] = (uint8_t) (-target_current[PLATE] >> 8);
        txmsg.data8[7] = (uint8_t) -target_current[PLATE];
    }


    can_->send_msg(&txmsg);

    // Set the PWM of friction wheels
    pwmEnableChannel(&PWMD8, FW_LEFT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, fw_duty_cycle * 500 + 500));
    pwmEnableChannel(&PWMD8, FW_RIGHT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, fw_duty_cycle * 500 + 500));

}

void GimbalIF::process_motor_feedback(CANRxFrame const *rxmsg) {

    /**
     * Function logic description:
     *  1. First, get the absolute angle value from the motor, compared with the last absolute angle value, get the
     *     angle movement
     *  2. Add the angle movement with the relative angle, get the new relative angle, modify the relative angle and
     *     the base round value
     */

    /// Get new absolute angle value of motor
    uint16_t new_actual_angle_raw = (rxmsg->data8[0] << 8 | rxmsg->data8[1]);

    // Check whether this new raw angle is valid
    if (new_actual_angle_raw > 8191) return;

    chSysLock();  /// ---------------------------------- Enter Critical Zone ----------------------------------

    motor_feedback_t* fb = &feedback[(motor_id_t) (rxmsg->SID - 0x205)];

    /// Calculate the angle movement in raw data
    // KEY IDEA: add the change of angle to actual angle
    // We assume that the absolute value of the angle movement is smaller than 180 degrees (4096 of raw data)
    int angle_movement = (int) new_actual_angle_raw - (int) fb->last_angle_raw;

    // Store new_actual_angle_raw for calculation of angle_movement next time
    fb->last_angle_raw = new_actual_angle_raw;

    /// If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0(8192) point
    if (angle_movement < -4096) angle_movement += 8192;
    if (angle_movement > 4096) angle_movement -= 8192;

    switch (fb->type) {

        case RM6623:  // RM6623 deceleration ratio = 1

            // raw -> degree
            fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

            fb->actual_velocity = 0;  // no velocity feedback available

            fb->actual_current = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);
            
            break;

        case M2006:  // M2006 deceleration ratio = 36,

            // raw -> degree with deceleration ratio
            fb->actual_angle += angle_movement * 0.001220703f;  // * 360 / 8192 / 36

            // rpm -> degree/s with deceleration ratio
            fb->actual_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) * 0.166666667f;  // 360 / 60 / 36

            fb->actual_current = 0;  // no current feedback available

            break;

        case M3508:  // M3508 deceleration ratio = 3591/187

            // raw -> degree with deceleration ratio
            fb->actual_angle += angle_movement * 0.002288436f; // 360 / 8192 / (3591/187)

            // rpm -> degree/s with deceleration ratio
            fb->actual_velocity =
                    ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) * 0.312447786f;  // 360 / 60 / (3591/187)

            fb->actual_current = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);

            break;

        case GM6020:  // GM6020 deceleration ratio = 1

            // raw -> degree
            fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

            // rpm -> degree/s
            fb->actual_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) * 6.0f;  // 360 / 60

            fb->actual_current = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);
            
            break;

        case GM3510:  // GM3510 deceleration ratio = 1

            // raw -> degree
            fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

            fb->actual_velocity = 0;  // no velocity feedback available

            fb->actual_current = 0;  // no current feedback available

        default:
            break;
    }

    /// Normalize the angle to [-180, 180]
    // If the actual_angle is greater than 180(-180) then it turns a round in CCW(CW) direction
    if (fb->actual_angle >= 180.0f) {
        fb->actual_angle -= 360.0f;
        fb->round_count++;
    }
    if (fb->actual_angle < -180.0f) {
        fb->actual_angle += 360.0f;
        fb->round_count--;
    }

    fb->last_update_time = SYSTIME;
    
    chSysUnlock();  /// ---------------------------------- Exit Critical Zone ----------------------------------

}

void GimbalIF::motor_feedback_t::init(motor_type_t type_, motor_id_t id_) {
    type = type_;
    id = id_;
}

void GimbalIF::motor_feedback_t::reset_front_angle() {
    actual_angle = 0;
    round_count = 0;
}

float GimbalIF::motor_feedback_t::accumulated_angle() {
    return actual_angle + round_count * 360.0f;
}

/** @} */