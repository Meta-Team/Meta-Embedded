//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback.
// Feng Chuhao wrote code about sending gimbal currents.
//

#include "gimbal_interface.h"
#include "common_macro.h"

GimbalInterface::MotorInterface GimbalInterface::yaw;
GimbalInterface::MotorInterface GimbalInterface::pitch;
GimbalInterface::MotorInterface GimbalInterface::bullet_loader;
GimbalInterface::FrictionWheelsInterface GimbalInterface::friction_wheels;
CANInterface *GimbalInterface::can_ = nullptr;
PWMConfig constexpr GimbalInterface::FRICTION_WHEELS_PWM_CFG;

void GimbalInterface::init(CANInterface *can_interface, uint16_t yaw_front_angle_raw, uint16_t pitch_front_angle_raw) {

    can_ = can_interface;
    can_->register_callback(0x205, 0x207, process_motor_feedback);

    yaw.id = YAW_ID;
    yaw.last_angle_raw = yaw_front_angle_raw;
    pitch.id = PIT_ID;
    pitch.last_angle_raw = pitch_front_angle_raw;
    bullet_loader.id = BULLET_LOADER_ID;
    bullet_loader.reset_front_angle();

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
}

bool GimbalInterface::send_gimbal_currents() {

    if (!can_) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x1FF;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    // Fill the current of Yaw
    if (yaw.enabled) {
#if GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(yaw.target_current, GIMBAL_INTERFACE_MAX_CURRENT);
#endif
        /** NOTICE: target current is reversed. */
        txmsg.data8[0] = (uint8_t) (-yaw.target_current >> 8); //upper byte
        txmsg.data8[1] = (uint8_t) -yaw.target_current; // lower byte

    } else {
        txmsg.data8[0] = txmsg.data8[1] = 0;
    }

    // Fill the current of Pitch
    if (pitch.enabled) {
#if GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(pitch.target_current, GIMBAL_INTERFACE_MAX_CURRENT);
#endif
        /** NOTICE: target current is reversed. */
        txmsg.data8[2] = (uint8_t) (-pitch.target_current >> 8); //upper byte
        txmsg.data8[3] = (uint8_t) -pitch.target_current; // lower byte

    } else {
        txmsg.data8[2] = txmsg.data8[3] = 0;
    }

    // Fill the current of bullet loader

    if (bullet_loader.enabled) {
#if GIMBAL_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(bullet_loader.target_current, GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT);
#endif
        txmsg.data8[4] = (uint8_t) (bullet_loader.target_current >> 8); //upper byte
        txmsg.data8[5] = (uint8_t) bullet_loader.target_current; // lower byte
    } else {
        txmsg.data8[4] = txmsg.data8[5] = 0;
    }

    txmsg.data8[6] = txmsg.data8[7] = 0;

    can_->send_msg(&txmsg);

    // Set the PWM of friction wheels
    if (friction_wheels.enabled) {
        pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_LEFT,
                         PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, friction_wheels.duty_cycle * 500 + 500));
        pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_RIGHT,
                         PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, friction_wheels.duty_cycle * 500 + 500));
    } else {
        pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_LEFT,
                         PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 0 * 500 + 500));
        pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, FW_RIGHT,
                         PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 0 * 500 + 500));
    }

    return true;
}

void GimbalInterface::process_motor_feedback(CANRxFrame const *rxmsg) {

    /*
     * function logic description
     * first, get the absolute angle value from the motor, compared with the last absolute angle value, get the angle movement
     * add the angle movement with the relative angle, get the new relative angle, modify the relative angle and the base round value
     * divide the angle movement by the time break to get the angular velocity
     */

    MotorInterface *motor = nullptr;

    uint16_t last_angle_raw;

    if (rxmsg->SID == 0x205) {
        motor = &yaw;
        last_angle_raw = motor->last_angle_raw;
    } else if (rxmsg->SID == 0x206) {
        motor = &pitch;
        last_angle_raw = motor->last_angle_raw;
    } else if (rxmsg->SID == 0x207) {
        last_angle_raw = bullet_loader.last_angle_raw;
    } else
        return;

    // Get the present absolute angle value by combining the data into a temporary variable
    uint16_t new_actual_angle_raw = (rxmsg->data8[0] << 8 | rxmsg->data8[1]);

    // Check whether this new raw angle is valid
    if (new_actual_angle_raw > 8191) {
        return;
    }

    // Calculate the angle movement in raw data
    // and we assume that the absolute value of the angle movement is smaller than 180 degrees(4096 of raw data)
    // currently motor->last_angle_raw holds the actual angle of last time
    int angle_movement = (int) new_actual_angle_raw - (int) last_angle_raw;

    switch (rxmsg->SID) {
        case 0x205:
        case 0x206: // If it is Yaw or Pitch

            // update the last_angle_raw
            motor->last_angle_raw = new_actual_angle_raw;

            // If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0(8192) point.
            if (angle_movement < -4096) {
                angle_movement += 8192;
            } else if (angle_movement > 4096) {
                angle_movement -= 8192;
            }

            // the key idea is to add the change of angle to actual angle.
            motor->actual_angle = motor->actual_angle + angle_movement * 360.0f / 8192;

            // modify the actual angle and update the round count when appropriate
            if (motor->actual_angle >= 180.0f) {
                //if the actual_angle is greater than 180, then we can know that it has already turn a round in counterclockwise direction
                motor->actual_angle -= 360.0f;//set the angle to be within [-180,180)
                motor->round_count++;//round count increases 1
            }
            if (motor->actual_angle < -180.0f) {
                // if the actual_angle is smaller than -180, then we can know that it has already turn a round in clockwise direction
                motor->actual_angle += 360.0f;//set the angle to be within [-180,180)
                motor->round_count--;//round count decreases 1
            }

            // Calculate the angular velocity and get the actual current
            // Sum angle movements for VELOCITY_SAMPLE_INTERVAL times, and calculate the average.
            motor->sample_movement_sum += angle_movement;
            motor->sample_count++;
            if (motor->sample_count >= VELOCITY_SAMPLE_INTERVAL) {
                // calculate the angular velocity
                time_msecs_t new_sample_time = TIME_I2MS(chibios_rt::System::getTime());
                motor->angular_velocity = motor->sample_movement_sum * 360.0f * 1000.0f / 8192.0f /
                                          (float) (new_sample_time - motor->sample_time);
                motor->sample_time = new_sample_time;

                motor->sample_movement_sum = 0;
                motor->sample_count = 0;
            }
            motor->actual_current = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);
            break;

        case 0x207: // If it is the bullet_loader

            // update the last_angle_raw
            bullet_loader.last_angle_raw = new_actual_angle_raw;

            // make sure that the angle movement is positive
            if (angle_movement < -2000) angle_movement = angle_movement + 8192;

            // the key idea is to add the change of angle to actual angle / 36 (deceleration ratio)
            bullet_loader.actual_angle = bullet_loader.actual_angle + angle_movement * 10.0f / 8192;

            // modify the actual angle and update the round count when appropriate
            if (bullet_loader.actual_angle >= 360.0f) {
                //if the actual_angle is greater than 360, then we can know that it has already turn a round in ***wise direction
                bullet_loader.actual_angle -= 360.0f;//set the angle to be within [0,360)
                bullet_loader.round_count++;//round count increases 1
            }
            if (bullet_loader.actual_angle < 0.0f) {
                // if the actual_angle is smaller than 0, then we can know that it has already turn a round in ***wise direction
                bullet_loader.actual_angle += 360.0f;//set the angle to be within [0,360)
                bullet_loader.round_count--;//round count decreases 1
            }

            // Get the angular velocity: feedback / 36 (deceleration ratio) * 6.0f (360 degrees per round per 60 seconds)
            bullet_loader.angular_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) /
                                             6.0f;

            break;

        default:
            break;
    }
}

void GimbalInterface::MotorInterface::reset_front_angle() {
    actual_angle = 0;
    round_count = 0;
}

float GimbalInterface::MotorInterface::get_accumulate_angle() {
    return actual_angle + round_count * 360.0f;
}