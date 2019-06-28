//
// Created by zhukerui on 2019/6/8.
//

#include "suspension_gimbal_interface.h"
#include "common_macro.h"

shoot_mode_t SuspensionGimbalIF::shoot_mode;
SuspensionGimbalIF::MotorInterface SuspensionGimbalIF::yaw;
SuspensionGimbalIF::MotorInterface SuspensionGimbalIF::pitch;
SuspensionGimbalIF::MotorInterface SuspensionGimbalIF::bullet_loader;
float SuspensionGimbalIF::pitchFront;
float SuspensionGimbalIF::shoot_duty_cycles[3] = {0.0, 0.1, 0.3};
CANInterface *SuspensionGimbalIF::can_;
AHRSExt *SuspensionGimbalIF::ahrs_;
PWMConfig constexpr SuspensionGimbalIF::FRICTION_WHEELS_PWM_CFG;

void SuspensionGimbalIF::init(CANInterface *can_interface, AHRSExt *ahrsExt, float yaw_front_angle_raw, float pitch_front_angle_raw) {
    shoot_mode = OFF;
    // The reasonable angle movement of yaw and pitch are assumed to be limited in (-4096, 4096), in other word, (-180, 180) in degree
    // The actual angle of yaw and pitch are scaled to be in [-180, 180) in degree

    yaw.initializer(YAW_ID, -180.0f, 180.0f, 1.0f);
    pitch.initializer(PIT_ID, -180.0f, 180.0f, 1.0f);

    // The bullet loader are assumed to only move in positive direction, so there is no real upper limit
    // The angle_movement_lower_bound here is to debug the mild turn back due to the unexpected vibration

    bullet_loader.initializer(BULLET_LOADER_ID, -1000, 400.0f, 36.0f);

    yaw.last_angle = yaw_front_angle_raw;
    pitch.last_angle = pitch_front_angle_raw;

    yaw.reset_front_angle();
    pitch.reset_front_angle();
    bullet_loader.reset_front_angle();

    can_ = can_interface;
    can_->register_callback(0x205, 0x207, process_motor_feedback);

    ahrs_ = ahrsExt;
    ahrs_->start(can_);

    pitchFront = ahrs_->angle.z;

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
        ABS_CROP(yaw.target_signal, SUSPENSION_YAW_MAX_VOLTAGE);
#endif
        txmsg.data8[0] = (uint8_t) (yaw.target_signal >> 8); //upper byte
        txmsg.data8[1] = (uint8_t) yaw.target_signal; // lower byte

    } else {
        txmsg.data8[0] = txmsg.data8[1] = 0;
    }

    // Fill the current of Pitch
    if (pitch.enabled) {
#if SUSPENSION_GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_CROP(pitch.target_signal, SUSPENSION_PITCH_MAX_VOLTAGE);
#endif
        /** NOTICE: target current is reversed. */
        txmsg.data8[2] = (uint8_t) (pitch.target_signal >> 8); //upper byte
        txmsg.data8[3] = (uint8_t) pitch.target_signal; // lower byte

    } else {
        txmsg.data8[2] = txmsg.data8[3] = 0;
    }

    // Fill the current of bullet loader

    if (bullet_loader.enabled && shoot_mode == SHOOT) {
#if SUSPENSION_GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_CROP(bullet_loader.target_signal, SUSPENSION_GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT);
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
LOG("%.2f", shoot_duty_cycles[shoot_mode]);
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
        pitch.angular_position = -(ahrs_->angle.z - pitchFront);
        pitch.angular_velocity = -ahrs_->gyro.y;
        return;
    } else{
        motor = &bullet_loader;
    }
    // Get the present absolute angle value by combining the data into a temporary variable
    float new_actual_angle = (rxmsg->data8[0] << 8 | rxmsg->data8[1]) * 360.0f / motor->deceleration_ratio / 8192.0f;

    // Check whether this new angle is valid
    if (new_actual_angle > 360.0f) {
        return;
    }

    // Calculate the angle movement
    float angle_movement = new_actual_angle - motor->last_angle;

    // update the last_angle
    motor->last_angle = new_actual_angle;

    // For angle movement, we take the smaller curve as the real movement
    if (angle_movement < motor->angle_movement_lower_bound) {
        angle_movement += 360.0f;
    } else if (angle_movement > motor->angle_movement_upper_bound) {
        angle_movement -= 360.0f;
    }
    // the key idea is to add the change of angle to actual angle.
    motor->angular_position = motor->angular_position + angle_movement;

    // Calculate the angular velocity
    // For yaw or bullet_loader, get the velocity directly
    // Get the angular velocity: feedback / deceleration ratio * 6.0f (360 degrees per round per 60 seconds)
    motor->angular_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) / motor->deceleration_ratio * 6.0f;
}