//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback.
// Feng Chuhao wrote code about sending gimbal currents.
//

#include "gimbal_interface.h"
#include "common_macro.h"

GimbalInterface::motor_feedback_t GimbalInterface::feedback[MOTOR_COUNT];
int GimbalInterface::target_current[MOTOR_COUNT] = {0, 0, 0};
float GimbalInterface::fw_duty_cycle = 0.0f;
CANInterface *GimbalInterface::can_ = nullptr;

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

void GimbalInterface::init(CANInterface *can_interface, uint16_t yaw_front_angle_raw, uint16_t pitch_front_angle_raw) {

    feedback[YAW].id = YAW;
    feedback[YAW].last_angle_raw = yaw_front_angle_raw;

    feedback[PITCH].id = PITCH;
    feedback[PITCH].last_angle_raw = pitch_front_angle_raw;

    feedback[BULLET].id = BULLET;
    feedback[BULLET].reset_front_angle();

    feedback[PLATE].id = PLATE;
    feedback[PLATE].reset_front_angle();

    can_ = can_interface;
    can_->register_callback(0x205, 0x208, process_motor_feedback);

#if defined(BOARD_RM_2018_A)
    // Enable power of bullet loader motor
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);
#endif


    // Enable PWM and perform initialization on friction wheels

    pwmStart(&PWMD8, &FRICTION_WHEELS_PWM_CFG);

    pwmEnableChannel(&PWMD8, FW_LEFT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1 * 500 + 500));
    pwmEnableChannel(&PWMD8, FW_RIGHT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1 * 500 + 500));
    chThdSleep(TIME_MS2I(500));
    pwmEnableChannel(&PWMD8, FW_LEFT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0 * 500 + 500));
    pwmEnableChannel(&PWMD8, FW_RIGHT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0 * 500 + 500));

}

void GimbalInterface::send_gimbal_currents() {

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
    /** NOTICE: target current is reversed. */
    txmsg.data8[0] = (uint8_t) (-target_current[YAW] >> 8); // upper byte
    txmsg.data8[1] = (uint8_t) -target_current[YAW];       // lower byte


    // Fill the current of Pitch
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[PITCH], GIMBAL_INTERFACE_MAX_CURRENT);
#endif
    /** NOTICE: target current is reversed. */
    txmsg.data8[2] = (uint8_t) (-target_current[PITCH] >> 8); // upper byte
    txmsg.data8[3] = (uint8_t) -target_current[PITCH];       // lower byte


    // Fill the current of bullet loader

#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[BULLET], GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT);
#endif
    txmsg.data8[4] = (uint8_t) (target_current[BULLET] >> 8); // upper byte
    txmsg.data8[5] = (uint8_t) target_current[BULLET];       // lower byte

    // Fill the current of plate

#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[PLATE], GIMBAL_INTERFACE_PLATE_MAX_CURRENT);
#endif
    txmsg.data8[6] = (uint8_t) (target_current[PLATE] >> 8);
    txmsg.data8[7] = (uint8_t) target_current[PLATE];


    can_->send_msg(&txmsg);

    // Set the PWM of friction wheels
    pwmEnableChannel(&PWMD8, FW_LEFT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, fw_duty_cycle * 500 + 500));
    pwmEnableChannel(&PWMD8, FW_RIGHT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, fw_duty_cycle * 500 + 500));

}

void GimbalInterface::process_motor_feedback(CANRxFrame const *rxmsg) {

    chSysLock();  // --- Enter Critical Zone ---

    /**
     * Function logic description:
     *  1. First, get the absolute angle value from the motor, compared with the last absolute angle value, get the
     *     angle movement
     *  2. Add the angle movement with the relative angle, get the new relative angle, modify the relative angle and
     *     the base round value
     *  3. Divide the angle movement by the time break to get the angular velocity
     */

    motor_id_t id = (motor_id_t) (rxmsg->SID - 0x205);

    // Get the present absolute angle value by combining the data into a temporary variable
    uint16_t new_actual_angle_raw = (rxmsg->data8[0] << 8 | rxmsg->data8[1]);

    // Check whether this new raw angle is valid
    if (new_actual_angle_raw > 8191) {
        chSysUnlock();  // --- Exit Critical Zone ---
        return;
    }

    // Calculate the angle movement in raw data
    // We assume that the absolute value of the angle movement is smaller than 180 degrees (4096 of raw data)

    int angle_movement = (int) new_actual_angle_raw - (int) feedback[id].last_angle_raw;

    switch (id) {
        case 0:
        case 1:  // Yaw or Pitch

            feedback[id].last_angle_raw = new_actual_angle_raw;

            // If angle_movement is too extreme between two samples,
            // we grant that it's caused by moving over the 0(8192) point.
            if (angle_movement < -4096) {
                angle_movement += 8192;
            } else if (angle_movement > 4096) {
                angle_movement -= 8192;
            }

            // KEY IDEA: add the change of angle to actual angle
            feedback[id].actual_angle += angle_movement * 360.0f / 8192;

            // If the actual_angle is greater than 180(-180) then it turns a round in CCW(CW) direction
            if (feedback[id].actual_angle >= 180.0f) {
                feedback[id].actual_angle -= 360.0f;
                feedback[id].round_count++;
            }
            if (feedback[id].actual_angle < -180.0f) {
                feedback[id].actual_angle += 360.0f;
                feedback[id].round_count--;
            }

#if GIMBAL_INTERFACE_ENABLE_VELOCITY_CALCULATION

            // Calculate the angular velocity and get the actual current
            // Sum angle movements for VELOCITY_SAMPLE_INTERVAL times, and calculate the average

            feedback[id].sample_movement_sum += angle_movement;
            feedback[id].sample_count++;
            if (feedback[id].sample_count >= VELOCITY_SAMPLE_INTERVAL) {

                time_msecs_t new_sample_time = SYSTIME;
                feedback[id].actual_velocity = feedback[id].sample_movement_sum * 360.0f * 1000.0f / 8192.0f /
                                               (float) (new_sample_time - feedback[id].sample_time_stamp);
                feedback[id].sample_time_stamp = new_sample_time;

                feedback[id].sample_movement_sum = 0;
                feedback[id].sample_count = 0;
            }
#endif

            feedback[id].actual_current = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);

            feedback[id].last_update_time = SYSTIME;

            break;

        case 2:  // Bullet Loader

            feedback[id].last_angle_raw = new_actual_angle_raw;

            // Make sure that the angle movement is positive
            if (angle_movement < -2000) angle_movement = angle_movement + 8192;

            // KEY IDEA: add the change of angle to actual angle / 36 (deceleration ratio)
            feedback[id].actual_angle += angle_movement * 10.0f / 8192;

            // If the actual_angle is greater than 180(-180) then it turns a round in CCW(CW) direction
            if (feedback[id].actual_angle >= 360.0f) {
                feedback[id].actual_angle -= 360.0f;
                feedback[id].round_count++;
            }
            if (feedback[id].actual_angle < 0.0f) {
                feedback[id].actual_angle += 360.0f;
                feedback[id].round_count--;
            }

            // Get the angular velocity: feedback / 36 (deceleration ratio) * 6.0f (360 degrees per round per 60 s)
            feedback[id].actual_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) / 6.0f;

            feedback[id].last_update_time = SYSTIME;

            break;

        case 3:   // PLATE

            feedback[id].last_angle_raw = new_actual_angle_raw;

            // Make sure that the angle movement is positive
            if (angle_movement < -2000) angle_movement = angle_movement + 8192;


            feedback[id].actual_angle += angle_movement * 18.9f / 8192; // deceleration ratio : 19, 360/19 = 18.947368421052632

            // IF the actual angle is beyond (-180,180)
            if (feedback[id].actual_angle >= 360.0f) {
                feedback[id].actual_angle -= 360.0f;
                feedback[id].round_count++;
            }
            if (feedback[id].actual_angle <= -360.0f) {
                feedback[id].actual_angle +=360.0f;
                feedback[id].round_count--;
            }

            feedback[id].actual_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) / 19.2f * 360.0f / 60.0f;

            feedback[id].last_update_time = SYSTIME;

            break;
        default:

            break;
    }

    chSysUnlock();  // --- Exit Critical Zone ---
}

void GimbalInterface::motor_feedback_t::reset_front_angle() {
    actual_angle = 0;
    round_count = 0;
}

float GimbalInterface::motor_feedback_t::get_accumulate_angle() {
    return actual_angle + round_count * 360.0f;
}
