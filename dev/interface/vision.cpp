//
// Created by Kerui Zhu on 7/4/2019.
//

#include "vision.h"
#include "CRC16.h"
#include "CRC8.h"
#include "shell.h"
#include "string.h"
#include "led.h"
#include "gimbal_scheduler.h"

float Vision::bullet_speed = 0;
time_msecs_t Vision::basic_gimbal_delay = 0;
time_msecs_t Vision::basic_shoot_delay = 0;
time_msecs_t Vision::shoot_tolerance = 0;
float Vision::target_armor_yaw = 0;
float Vision::target_armor_pitch = 0;
Vision::LowPassFilteredValue Vision::target_armor_dist;
float Vision::last_gimbal_yaw = 0;
float Vision::last_gimbal_pitch = 0;
time_msecs_t Vision::last_update_time = 0;
time_msecs_t Vision::last_update_delta = 0;
Vision::VelocityCalculator Vision::velocity_calculator;
bool Vision::can_reach_the_target = false;
int Vision::flight_time_to_target = 0;
Vision::GimbalCommand Vision::latest_gimbal_command;
int Vision::expected_shoot_time = 0;
int Vision::expected_shoot_after_periods = 0;
constexpr size_t Vision::DATA_SIZE[Vision::CMD_ID_COUNT];

Vision::package_t Vision::pak;
Vision::rx_status_t Vision::rx_status;

const UARTConfig Vision::UART_CONFIG = {
        nullptr,
        nullptr,
        Vision::uart_rx_callback,  // callback function when the buffer is filled
        Vision::uart_char_callback,
        Vision::uart_err_callback,
        115200, // speed
        0,
        0,
        0
};

void Vision::init(float distance_filter_alpha, time_msecs_t basic_gimbal_delay_, time_msecs_t basic_shoot_delay_,
                  time_msecs_t shoot_tolerance_) {

    target_armor_dist.set_alpha(distance_filter_alpha);
    basic_gimbal_delay = basic_gimbal_delay_;
    basic_shoot_delay = basic_shoot_delay_;
    shoot_tolerance = shoot_tolerance_;

    // Start UART driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    uartStartReceive(UART_DRIVER, sizeof(uint8_t), &pak);
}

bool Vision::should_update_gimbal(GimbalCommand &command) {
    bool ret;
    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    {
        if (WITHIN_RECENT_TIME(last_update_time, 1000)) {
            if (!can_reach_the_target) {
                ret = false;
            } else {
                command = latest_gimbal_command;
                ret = true;
            }
        } else {
            velocity_calculator.reset();
            ret = false;
        }
    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
    return ret;
}

bool Vision::should_shoot() {
    if (!WITHIN_RECENT_TIME(last_update_time, 1000)) return true;  // vision command invalid, shoot anytime
    if (!can_reach_the_target) return false;  // you can not reach the target...
    if (expected_shoot_time == 0) return true;  // free fire
    if (ABS((int) (SYSTIME) - (int) (expected_shoot_time)) <= (int) shoot_tolerance) {
        return true;  // if shoot now, we expect to hit the target
    } else {
        return false;
    }
}

void Vision::predict_target_position(float &yaw, float &pitch, float &dist) {
    float rough_flight_time = dist / bullet_speed;  // not accurate, but probably enough
    float predict_forward_amount = (float) basic_gimbal_delay + rough_flight_time;
    yaw += velocity_calculator.yaw_velocity() * predict_forward_amount;
    (void) pitch;
    /* pitch += velocity_calculator.pitch_velocity() * predict_forward_amount; */
    (void) dist;
    /* dist += velocity_calculator.dist_velocity() * predict_forward_amount; */
}

inline float pow2(float v) { return v * v; }

#ifndef PI
#define PI 3.14159265358979f
#endif

bool Vision::compensate_for_gravity(float &pitch, float dist, int &flight_time) {
    float a = pow2(bullet_speed) / g;
    float y = dist * -sinf(pitch * PI / 180.0f);  // + for upward [mm]
    float b = pow2(a - y) - pow2(dist);
    if (b < 0) return false;
    float c = a - sqrtf(b);
    pitch = -atanf(c / (dist * cosf(pitch * PI / 180.0f))) * 180.0f / PI;
    flight_time = (int) sqrtf((c - y) / (0.5f * g));
    return true;
}

void Vision::handle_vision_command(const vision_command_t &command) {
    // This function is called in I-Locked state. DO NOT use LOG, printf, non I-Class functions.

    time_msecs_t now = SYSTIME;

    if (command.flag & DETECTED) {

        if (last_update_time == 0 || !WITHIN_DURATION(now, last_update_time, 1000)) {
            // No previous data or out-of-date, use the latest data
            last_gimbal_yaw = GimbalSKD::get_accumulated_angle(GimbalSKD::YAW);
            last_gimbal_pitch = GimbalSKD::get_accumulated_angle(GimbalSKD::PITCH);
        }  // otherwise,  Use gimbal angles at last command, which is roughly the angles when the image is captured

        float new_armor_yaw = ((float) command.yaw_delta / 100.0f) + last_gimbal_yaw;
        float new_armor_pitch = ((float) command.pitch_delta / 100.0f) + last_gimbal_pitch;
        if (command.flag & TOP_KILLER_TRIGGERED) {
            target_armor_dist.direct_set(command.dist);
            velocity_calculator.reset();
        } else {
            target_armor_dist.update(command.dist);
            velocity_calculator.update(new_armor_yaw, new_armor_pitch, target_armor_dist.get(), now);
        }

        // Store latest armor position if detected
        target_armor_yaw = new_armor_yaw;
        target_armor_pitch = new_armor_pitch;

        // Predict and compensate
        float yaw = target_armor_yaw, pitch = target_armor_pitch, dist = target_armor_dist.get();
        predict_target_position(yaw, pitch, dist);
        can_reach_the_target = compensate_for_gravity(pitch, dist, flight_time_to_target);
        if (can_reach_the_target) {

            latest_gimbal_command = {yaw, pitch};

            // Predict time to shootx
            if (command.flag & TOP_KILLER_TRIGGERED) {
                expected_shoot_time = (int) last_update_time
                                      + (int) command.remainingTimeToTarget
                                      - (int) flight_time_to_target
                                      - (int) basic_shoot_delay;
                expected_shoot_after_periods = 0;
                while (true) {
                    int time_delta = (int) expected_shoot_time - (int) now;
                    if (time_delta < 0) {
                        expected_shoot_after_periods++;
                        expected_shoot_time += command.period;
                    } else {
                        break;
                    }
                }
            } else {
                expected_shoot_time = 0;  // shoot anytime
            }

        }
    }

    // Record current gimbal angles, which will be used for next control command
    last_gimbal_yaw = GimbalSKD::get_accumulated_angle(GimbalBase::YAW);
    last_gimbal_pitch = GimbalSKD::get_accumulated_angle(GimbalBase::PITCH);
    last_update_delta = now - last_update_time;
    last_update_time = now;
}

void Vision::VelocityCalculator::update(float armor_yaw, float armor_pitch, float armor_dist, time_msecs_t time) {
    if (last_compute_time == 0) {
        last_yaw = armor_yaw;
        last_pitch = armor_pitch;
        last_dist = armor_dist;
        last_compute_time = time;
    } else if (time - last_compute_time > MIN_COMPUTE_INTERNAL) {
        yaw_v.update((armor_yaw - last_yaw) / (float) (time - last_compute_time));
        pitch_v.update((armor_pitch - last_pitch) / (float) (time - last_compute_time));
        dist_v.update((armor_dist - last_dist) / (float) (time - last_compute_time));
        last_yaw = armor_yaw;
        last_pitch = armor_pitch;
        last_dist = armor_dist;
        last_compute_time = time;
    }
}

void Vision::VelocityCalculator::reset() {
    last_compute_time = 0;
    last_yaw = last_pitch = last_dist = 0;
    yaw_v.direct_set(0);
    pitch_v.direct_set(0);
    dist_v.direct_set(0);
}

void Vision::uart_rx_callback(UARTDriver *uartp) {
    (void) uartp;

    chSysLockFromISR();  /// --- ENTER I-Locked state. DO NOT use LOG, printf, non I-Class functions or return ---

    auto pak_uint8 = (uint8_t *) &pak;

#ifdef VISION_PORT_DEBUG
    LED::red_toggle();
#warning "VisionPort: in debug mode now"
#endif

    switch (rx_status) {

        case WAIT_STARTING_BYTE:
            if (pak_uint8[0] == 0xA5) {
                rx_status = WAIT_CMD_ID;
            } // otherwise, keep waiting for SOF
            break;

        case WAIT_CMD_ID:
            if (pak.cmd_id < CMD_ID_COUNT) {
                rx_status = WAIT_DATA_TAIL; // go to next status
            } else {
                rx_status = WAIT_STARTING_BYTE;
            }
            break;

        case WAIT_DATA_TAIL:

            if (Verify_CRC8_Check_Sum(pak_uint8, sizeof(uint8_t) * 2 + DATA_SIZE[pak.cmd_id] + sizeof(uint8_t))) {
                switch (pak.cmd_id) {
                    case VISION_CONTROL_CMD_ID: {
                        handle_vision_command(pak.command);

                        // Indicate receiving
                        LED::green_toggle();
                        break;
                    }
                }
            }
            rx_status = WAIT_STARTING_BYTE;
            break;
    }

    switch (rx_status) {
        case WAIT_STARTING_BYTE:
            uartStartReceiveI(uartp, sizeof(uint8_t), pak_uint8);
            break;
        case WAIT_CMD_ID:
            uartStartReceiveI(uartp, sizeof(uint8_t), pak_uint8 + sizeof(uint8_t));
            break;
        case WAIT_DATA_TAIL:
            uartStartReceiveI(uartp, DATA_SIZE[pak.cmd_id] + sizeof(uint8_t), pak_uint8 + sizeof(uint8_t) * 2);
            break;
    }

    chSysUnlockFromISR();  /// --- EXIT I-Locked state ---
}

void Vision::uart_err_callback(UARTDriver *uartp, uartflags_t e) {
    (void) uartp;
    (void) e;
    LED::red_toggle();
}

void Vision::uart_char_callback(UARTDriver *uartp, uint16_t c) {
    (void) uartp;
    (void) c;
}