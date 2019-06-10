//
// Created by zhukerui on 2019/6/8.
//

#include "suspension_gimbal_interface.h"
#include "common_macro.h"

SuspensionGimbalIF::MotorInterface SuspensionGimbalIF::yaw;
SuspensionGimbalIF::MotorInterface SuspensionGimbalIF::pitch;
SuspensionGimbalIF::MotorInterface SuspensionGimbalIF::bullet_loader;
CANInterface *SuspensionGimbalIF::can_ = nullptr;
SuspensionGimbalIF::shoot_mode_t SuspensionGimbalIF::shoot_mode = OFF;
float SuspensionGimbalIF::shoot_duty_cycles[3] = {0.0, 0.1, 0.3};
PWMConfig constexpr SuspensionGimbalIF::FRICTION_WHEELS_PWM_CFG;

void SuspensionGimbalIF::init(CANInterface *can_interface, uint16_t yaw_front_angle_raw, uint16_t pitch_front_angle_raw) {
    yaw.id = YAW_ID;
    pitch.id = PIT_ID;
    // The reasonable angle movement of yaw and pitch are assumed to be limited in (-4096, 4096), in other word, (-180, 180) in degree
    // The actual angle of yaw and pitch are scaled to be in [-180, 180) in degree

    yaw.angle_movement_lower_bound = pitch.angle_movement_lower_bound = -4096;
    yaw.angle_movement_upper_bound = pitch.angle_movement_upper_bound = 4096;
    yaw.actual_angle_lower_bound = pitch.actual_angle_lower_bound = -180.0f;
    yaw.actual_angle_upper_bound = pitch.actual_angle_upper_bound = 180.0f;
    yaw.deceleration_ratio = pitch.deceleration_ratio = 1.0f;

    yaw.last_angle_raw = yaw_front_angle_raw;
    pitch.last_angle_raw = pitch_front_angle_raw;

    bullet_loader.id = BULLET_LOADER_ID;
    // The bullet loader are assumed to only move in positive direction, so there is no real upper limit
    // The angle_movement_lower_bound here is to debug the mild turn back due to the unexpected vibration

    bullet_loader.angle_movement_lower_bound = -1000;
    bullet_loader.angle_movement_upper_bound = 8193;
    bullet_loader.actual_angle_lower_bound = 0.0f;
    bullet_loader.actual_angle_upper_bound = 360.0f;
    bullet_loader.deceleration_ratio = 36.0f;

    bullet_loader.reset_front_angle();

    can_ = can_interface;
    can_->register_callback(0x205, 0x207, process_motor_feedback);

    pwmStart(FRICTION_WHEEL_PWM_DRIVER, &FRICTION_WHEELS_PWM_CFG);

    // Perform the initialization work of friction wheel driver (100% and then 0%)
    pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_LEFT,
                     PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 1 * 500 + 500));
    pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_RIGHT,
                     PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 1 * 500 + 500));
    chThdSleep(TIME_MS2I(500));
    pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_LEFT,
                     PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 0 * 500 + 500));
    pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_RIGHT,
                     PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 0 * 500 + 500));

#if defined(BOARD_RM_2018_A)
    // Enable power of bullet loader motor
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);
#endif
}

bool SuspensionGimbalIF::send_gimbal_currents() {

    if (!can_) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x1FF;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    // Fill the current of Yaw
    if (yaw.enabled) {
#if SUSPENSION_GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(yaw.target_signal, SUSPENSION_YAW_MAX_VOLTAGE);
#endif
        /** NOTICE: target current is reversed. */
        txmsg.data8[0] = (uint8_t) (-yaw.target_signal >> 8); //upper byte
        txmsg.data8[1] = (uint8_t) -yaw.target_signal; // lower byte

    } else {
        txmsg.data8[0] = txmsg.data8[1] = 0;
    }

    // Fill the current of Pitch
    if (pitch.enabled) {
#if SUSPENSION_GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(pitch.target_signal, SUSPENSION_PITCH_MAX_VOLTAGE);
#endif
        /** NOTICE: target current is reversed. */
        txmsg.data8[2] = (uint8_t) (-pitch.target_signal >> 8); //upper byte
        txmsg.data8[3] = (uint8_t) -pitch.target_signal; // lower byte

    } else {
        txmsg.data8[2] = txmsg.data8[3] = 0;
    }

    // Fill the current of bullet loader

    if (bullet_loader.enabled) {
#if SUSPENSION_GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(bullet_loader.target_signal, SUSPENSION_GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT);
#endif
        txmsg.data8[4] = (uint8_t) (bullet_loader.target_signal >> 8); //upper byte
        txmsg.data8[5] = (uint8_t) bullet_loader.target_signal; // lower byte
        //LOG("%d",bullet_loader.target_signal);
    } else {
        txmsg.data8[4] = txmsg.data8[5] = 0;
    }

    txmsg.data8[6] = txmsg.data8[7] = 0;

    can_->send_msg(&txmsg);

    // Set the PWM of friction wheels
    pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_LEFT,
                     PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, shoot_duty_cycles[shoot_mode] * 500 + 500));
    pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_RIGHT,
                     PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, shoot_duty_cycles[shoot_mode] * 500 + 500));

    return true;
}

void SuspensionGimbalIF::process_motor_feedback(CANRxFrame const *rxmsg) {

    /*
     * function logic description
     * first, get the absolute angle value from the motor, compared with the last absolute angle value, get the reasonable angle movement
     * add the angle movement with the relative angle, get the new relative angle, modify the relative angle and the base round value
     */

    MotorInterface* motor;
    if(rxmsg->SID == 0x205){
        motor = &yaw;
    }else if(rxmsg->SID == 0x206){
        motor = &pitch;
    } else{
        motor = &bullet_loader;
    }
    // Get the present absolute angle value by combining the data into a temporary variable
    uint16_t new_actual_angle_raw = (rxmsg->data8[0] << 8 | rxmsg->data8[1]);

    // Check whether this new raw angle is valid
    if (new_actual_angle_raw > 8191) {
        return;
    }

    // Calculate the angle movement in raw data
    int angle_movement = (int) new_actual_angle_raw - (int) motor->last_angle_raw;

    // update the last_angle_raw
    motor->last_angle_raw = new_actual_angle_raw;

    // If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0(8192) point.
    if (angle_movement < motor->angle_movement_lower_bound) {
        angle_movement += 8192;
    } else if (angle_movement > motor->angle_movement_upper_bound) {
        angle_movement -= 8192;
    }
    // the key idea is to add the change of angle to actual angle.
    motor->actual_angle = motor->actual_angle + angle_movement * 360.0f / motor->deceleration_ratio / 8192;

    // modify the actual angle and update the round count when appropriate
    if (motor->actual_angle >= motor->actual_angle_upper_bound) {
        motor->actual_angle -= 360.0f;
        motor->round_count++;//round count increases 1
    }
    if (motor->actual_angle < motor->actual_angle_lower_bound) {
        motor->actual_angle += 360.0f;
        motor->round_count--;//round count decreases 1
    }

    // Calculate the angular velocity
    if(rxmsg->SID == 0x205 || rxmsg->SID == 0x207) {
        // For yaw or bullet_loader, get the velocity directly
        // Get the angular velocity: feedback / deceleration ratio * 6.0f (360 degrees per round per 60 seconds)
        motor->angular_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) / motor->deceleration_ratio * 6.0f;
    }else{
        // For pitch, sum angle movements for VELOCITY_SAMPLE_INTERVAL times, and calculate the average.
        static int sample_count = 0;
        static int sample_movement_sum = 0;
        static time_msecs_t sample_time = 0;
        sample_movement_sum += angle_movement;
        sample_count++;
        if (sample_count >= VELOCITY_SAMPLE_INTERVAL) {
            // calculate the angular velocity
            auto new_sample_time = TIME_I2MS(chibios_rt::System::getTime());
            motor->angular_velocity = sample_movement_sum * 360.0f * 1000.0f / 8192.0f /
                                      (float) (new_sample_time - sample_time);
            sample_time = new_sample_time;

            sample_movement_sum = 0;
            sample_count = 0;
        }
    }
}