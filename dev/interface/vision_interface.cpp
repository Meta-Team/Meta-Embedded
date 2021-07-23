//
// Created by Kerui Zhu on 7/4/2019.
//

#include "vision_interface.h"
#include "CRC16.h"
#include "CRC8.h"
#include "shell.h"
#include "string.h"
#include "led.h"
#include "gimbal_scheduler.h"
#include "trajectory_calculator.hpp"

EVENTSOURCE_DECL(Vision::gimbal_updated_event);
EVENTSOURCE_DECL(Vision::shoot_time_updated_event);
float Vision::bullet_speed = 0;
time_msecs_t Vision::basic_gimbal_delay = 0;
PositionKalmanFilter Vision::armor_ypd[3];
constexpr float Vision::ARMOR_Q_POSITION[3];
constexpr float Vision::ARMOR_Q_VELOCITY[3];
constexpr float Vision::ARMOR_R_POSITION[3];
float Vision::last_gimbal_yaw = 0;
float Vision::last_gimbal_pitch = 0;
uint16_t Vision::last_frame_timestamp = 0;
time_msecs_t Vision::last_update_time = 0;
time_msecs_t Vision::last_update_time_delta = 0;
bool Vision::can_reach_target = false;
float Vision::bullet_flight_time = 0;
float Vision::latest_target_yaw = 0;
float Vision::latest_target_pitch = 0;
time_msecs_t Vision::expected_shoot_time = 0;
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

void Vision::init(time_msecs_t basic_gimbal_delay_) {

    for (int i = 0; i < 3; i++) {
        armor_ypd[i].set_Q_position(ARMOR_Q_POSITION[i]);
        armor_ypd[i].set_Q_velocity(ARMOR_Q_VELOCITY[i]);
        armor_ypd[i].set_R_position(ARMOR_R_POSITION[i]);
    }

    basic_gimbal_delay = basic_gimbal_delay_;

    // Start UART driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    uartStartReceive(UART_DRIVER, sizeof(uint8_t), &pak);
}

bool Vision::get_gimbal_target_angles_S(float &yaw, float &pitch) {
    if (WITHIN_RECENT_TIME(last_update_time, 1000)) {
        if (!can_reach_target) {
            return false;
        } else {
            yaw = latest_target_yaw;
            pitch = latest_target_pitch;
            return true;
        }
    } else {
        return false;
    }
}

void Vision::handle_vision_command(const vision_command_t &command) {
    /// This function is called in I-Locked state. DO NOT use LOG, printf, non I-Class functions.

    time_msecs_t now = SYSTIME;

    if (command.flag & DETECTED) {

        if (WITHIN_DURATION(now, last_update_time, 200)) {

            // Compute absolute angles
            float new_armor_yaw = ((float) command.yaw_delta / 100.0f) + last_gimbal_yaw;
            float new_armor_pitch = ((float) command.pitch_delta / 100.0f) + last_gimbal_pitch;

            float time_delta = (float) (command.time_stamp - last_frame_timestamp) / 10.0f;

            // Update Kalman filters
            armor_ypd[0].update(new_armor_yaw, time_delta);
            armor_ypd[1].update(new_armor_pitch, time_delta);
            armor_ypd[2].update(command.dist, time_delta);

            float yaw = armor_ypd[0].get_position();
            float pitch = armor_ypd[1].get_position();
            float dist = armor_ypd[2].get_position();

            // Predict movement
            {
                float rough_flight_time = dist / bullet_speed;  // not accurate, but probably enough
                float predict_forward_amount = (float) basic_gimbal_delay + rough_flight_time;
                yaw += armor_ypd[0].get_velocity() * predict_forward_amount;
            }

            // Compensate for gravity
            can_reach_target = Trajectory::compensate_for_gravity(pitch, dist, bullet_speed, bullet_flight_time);

            if (can_reach_target) {

                // Issue new gimbal command
                latest_target_yaw = yaw;
                latest_target_pitch = pitch;
                chEvtBroadcastI(&gimbal_updated_event);  // we are still in I-Lock state

                // Predict time to shoot
                /*if (command.flag & TOP_KILLER_TRIGGERED) {

                    // Calculate using time_msecs_t so that we don't need to care about wrap around
                    time_msecs_t shoot_time = last_update_time
                                              + (time_msecs_t) command.remaining_time_to_target
                                              - (time_msecs_t) flight_time_to_target
                                              - (time_msecs_t) measured_shoot_delay.get();

                    // Compensate for one or more periods until we can catch up expected_shoot_time
                    expected_shoot_after_periods = 0;
                    while (true) {
                        // Minus using time_msecs_t so that we don't need to care about wrap around
                        auto time_delta = (int32_t) (shoot_time - now);  // compare by casting to signed
                        if (time_delta < 0) {
                            expected_shoot_after_periods++;
                            shoot_time += command.period;
                        } else {
                            break;
                        }
                    }

                    // Issue shoot command
                    expected_shoot_time = (time_msecs_t) shoot_time;
                    chEvtBroadcastI(&shoot_time_updated_event);  // we are still in I-Lock state

                } else {
                    expected_shoot_time = 0;
                }*/
            }
        }
    }

    // Record current gimbal angles, which will be used for next control command
    last_gimbal_yaw = GimbalSKD::get_actual_angle(GimbalBase::YAW);
    last_gimbal_pitch = GimbalSKD::get_actual_angle(GimbalBase::PITCH);
    last_frame_timestamp = command.time_stamp;
    last_update_time_delta = now - last_update_time;
    last_update_time = now;
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

            if (verify_crc8_check_sum(pak_uint8, sizeof(uint8_t) * 2 + DATA_SIZE[pak.cmd_id] + sizeof(uint8_t))) {
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