//
// Created by Kerui Zhu on 7/4/2019.
//

#include "vision_port.h"
#include "CRC16.h"
#include "CRC8.h"
#include "shell.h"
#include "memstreams.h"
#include "string.h"
#include "led.h"
#include "gimbal_scheduler.h"

float Vision::last_gimbal_yaw = 0;
float Vision::last_gimbal_pitch = 0;
float Vision::target_armor_yaw = 0;
float Vision::target_armor_pitch = 0;
time_msecs_t Vision::last_update_time = 0;
float Vision::latest_yaw_velocity = 0;
float Vision::latest_pitch_velocity = 0;
float Vision::velocity_update_fraction = 1;
time_msecs_t Vision::predict_forward_amount = 0;
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

void Vision::init(float velocity_update_fraction_, time_msecs_t predict_forward_amount_) {

    velocity_update_fraction = velocity_update_fraction_;
    predict_forward_amount = predict_forward_amount_;

    // Start UART driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    uartStartReceive(UART_DRIVER, sizeof(uint8_t), &pak);
}

bool Vision::getControlCommand(VisionControlCommand &command) {
    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    bool ret;
    if (WITHIN_RECENT_TIME(last_update_time, 1000)) {
        command = {target_armor_yaw + latest_yaw_velocity * (float) (predict_forward_amount),
                target_armor_pitch /*+ latest_pitch_velocity * (float) (predict_forward_amount)*/};
        ret = true;
    } else {
        ret = false;
    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
    //LOG("%f", latest_yaw_velocity * (float) (predict_forward_amount));
    return ret;
}


void Vision::uart_rx_callback(UARTDriver *uartp) {
    (void) uartp;

    chSysLockFromISR();  /// --- ENTER I-Locked state. DO NOT use LOG, printf, non I-Class functions or return ---

    uint8_t *pak_uint8 = (uint8_t *) &pak;

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
                        if (last_update_time == 0 || !WITHIN_RECENT_TIME(last_update_time, 1000)) {
                            // No previous data or out-of-date, use the latest data
                            last_gimbal_yaw = GimbalSKD::get_accumulated_angle(GimbalSKD::YAW);
                            last_gimbal_pitch = GimbalSKD::get_accumulated_angle(GimbalSKD::PITCH);
                        }  // otherwise, use gimbal angles at last time

                        // These two data is only valid for detected
                        /*
                         * Use gimbal angles at last control command, which is roughly the angles when the
                         * image is captured.
                         */
                        float new_armor_yaw = (pak.command.yaw_delta / 100.0f) + last_gimbal_yaw;
                        float new_armor_pitch = (pak.command.pitch_delta / 100.0f) + last_gimbal_pitch;

                        float new_yaw_velocity, new_pitch_velocity;
                        if (pak.command.flag & DETECTED) {
                            new_yaw_velocity = (new_armor_yaw - target_armor_yaw) / (float) (SYSTIME - last_update_time);
                            new_pitch_velocity = (new_armor_pitch - target_armor_pitch) / (float) (SYSTIME - last_update_time);
                        } else {
                            new_yaw_velocity = new_pitch_velocity = 0;
                        }

                        // Update velocity
                        latest_yaw_velocity = new_yaw_velocity * velocity_update_fraction +
                                latest_yaw_velocity * (1 - velocity_update_fraction);
                        latest_pitch_velocity = new_pitch_velocity * velocity_update_fraction +
                                latest_pitch_velocity * (1 - velocity_update_fraction);

                        // Store latest armor position if detected
                        if (pak.command.flag & DETECTED) {
                            target_armor_yaw = new_armor_yaw;
                            target_armor_pitch = new_armor_pitch;
                        }

                        // Record current gimbal angles, which will be used for next control command
                        last_gimbal_yaw = GimbalSKD::get_accumulated_angle(GimbalBase::YAW);
                        last_gimbal_pitch = GimbalSKD::get_accumulated_angle(GimbalBase::PITCH);
                        last_update_time = SYSTIME;

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