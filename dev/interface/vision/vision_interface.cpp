//
// Created by Kerui Zhu on 7/4/2019.
//

#include "vision_interface.h"
#include "CRC16.h"
#include "CRC8.h"
#include "shell.h"
#include "led.h"
#include "gimbal_scheduler.h"

EVENTSOURCE_DECL(VisionIF::command_received_event);

VisionIF::vision_command_t VisionIF::latest_valid_command;
float VisionIF::latest_armor_yaw = 0;
float VisionIF::latest_armor_pitch = 0;

float VisionIF::last_gimbal_yaw = 0;
float VisionIF::last_gimbal_pitch = 0;

time_msecs_t VisionIF::last_command_receive_time = 0;
time_msecs_t VisionIF::last_valid_update_time = 0;
time_msecs_t VisionIF::last_valid_update_time_delta = 0;

constexpr size_t VisionIF::DATA_SIZE[VisionIF::CMD_ID_COUNT];

VisionIF::package_t VisionIF::pak;
VisionIF::rx_status_t VisionIF::rx_status;

const UARTConfig VisionIF::UART_CONFIG = {
        VisionIF::uart_txend_callback,
        nullptr,
        VisionIF::uart_rx_callback,  // callback function when the buffer is filled
        VisionIF::uart_char_callback,
        VisionIF::uart_err_callback,
        115200, // speed
        0,
        0,
        0
};
__PACKED_STRUCT VISIONFEEDBACK{
    uint8_t startOfFrame = 0x5A;
    float yaw,pitch;
    uint16_t crc16;
};
__PACKED_STRUCT VISION_RECV_YP{
    uint8_t startOfFrame = 0x5A;
    float yaw,pitch;
    uint8_t crc8;
};
VISIONFEEDBACK visionFeedback={
        0x5A,
        0.0f,
        0.0f,
        0
};
VISION_RECV_YP visionRecvYp={
        0x00,
        0.0f,
        0.0f,
        0
};
time_msecs_t lastRecvTime = 0;
THD_WORKING_AREA(VisionSend_wa, 256);
THD_FUNCTION(VisionSend,arg){
    while(1){
        visionFeedback.pitch = GimbalSKD::get_feedback_angle(GimbalSKD::PITCH);
        visionFeedback.yaw = GimbalSKD::get_feedback_angle(GimbalSKD::YAW);
        visionFeedback.crc16 = get_crc16_check_sum((uint8_t*)&visionFeedback,9);
        uartStartSend(&UARTD8,sizeof(VISIONFEEDBACK),&visionFeedback);
        chThdSleepMilliseconds(5);
    }
}
void VisionIF::init() {
    // Start UART driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    chThdCreateStatic(VisionSend_wa,sizeof(VisionSend_wa),NORMALPRIO,VisionSend,(void*)0);
#if 0
    uartStartReceive(UART_DRIVER, sizeof(uint8_t), &pak);
#else
    uartStartReceive(UART_DRIVER, sizeof(uint8_t), &visionRecvYp);
#endif
}

void VisionIF::get_latest_valid_command(VisionIF::vision_command_t &command,
                                        float &absolute_yaw, float &absolute_pitch) {

    command = latest_valid_command;
    absolute_yaw = latest_armor_yaw;
    absolute_pitch = latest_armor_pitch;

}

void VisionIF::handle_vision_command(const vision_command_t &command) {
    /// This function is called in I-Locked state. DO NOT use LOG, printf, non I-Class functions.

    time_msecs_t now = SYSTIME;

    if (WITHIN_DURATION(now, last_command_receive_time, 200)) {  // otherwise, last_gimbal_yaw/pitch is not valid

        if (command.flag & DETECTED) {

            // Compute absolute angles
            latest_valid_command = command;
            latest_armor_yaw = ((float) command.yaw_delta / 100.0f) + last_gimbal_yaw;
            latest_armor_pitch = ((float) command.pitch_delta / 100.0f) + last_gimbal_pitch;

            last_valid_update_time_delta = now - last_valid_update_time;
            last_valid_update_time = now;

            chEvtBroadcastI(&command_received_event);  // we are still in I-Lock state
        }
    }

    // Record current gimbal angles, which will be used for next control command
    last_gimbal_yaw = GimbalSKD::get_feedback_angle(GimbalSKD::YAW);
    last_gimbal_pitch = GimbalSKD::get_feedback_angle(GimbalSKD::PITCH);
    last_command_receive_time = now;
}

void VisionIF::uart_rx_callback(UARTDriver *uartp) {
    (void) uartp;

    chSysLockFromISR();  /// --- ENTER I-Locked state. DO NOT use LOG, printf, non I-Class functions or return ---
    {
        auto pak_uint8 = (uint8_t *) &visionRecvYp;

        switch (rx_status) {

            case WAIT_STARTING_BYTE:
                if (pak_uint8[0] == 0xA5) {
                    rx_status = WAIT_DATA_TAIL;
                } // otherwise, keep waiting for SOF
                break;
#if 0
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
#else
// Receive Yaw and Pitch data from ROS2 server
            case WAIT_DATA_TAIL:

                if (verify_crc8_check_sum(pak_uint8, sizeof(VISION_RECV_YP) + sizeof(uint8_t))) {
                    lastRecvTime = SYSTIME;
                    // set target Yaw and Pitch
                }
                rx_status = WAIT_STARTING_BYTE;
                break;
        }

        switch (rx_status) {
            case WAIT_STARTING_BYTE:
                uartStartReceiveI(uartp, 1, pak_uint8);
                break;
            case WAIT_DATA_TAIL:
                uartStartReceiveI(uartp, sizeof(VISION_RECV_YP)-1, pak_uint8 + 1);
                break;
        }
#endif
    }
    chSysUnlockFromISR();  /// --- EXIT I-Locked state ---
}

void VisionIF::uart_err_callback(UARTDriver *uartp, uartflags_t e) {
    (void) uartp;
    (void) e;
    LED::red_toggle();
}

void VisionIF::uart_char_callback(UARTDriver *uartp, uint16_t c) {
    (void) uartp;
    (void) c;
}
void VisionIF::uart_txend_callback(UARTDriver *uartp) {
    (void) uartp;
}