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

float VisionPort::last_gimbal_yaw = 0;
float VisionPort::last_gimbal_pitch = 0;
float VisionPort::vision_yaw_target = 0;
float VisionPort::vision_pitch_target = 0;
time_msecs_t VisionPort::last_update_time = 0;

VisionPort::package_t VisionPort::pak;
VisionPort::package_t VisionPort::tx_pak;
VisionPort::rx_status_t VisionPort::rx_status;

uint16_t VisionPort::tx_seq = 0;
VisionPort::TXThread VisionPort::txThread;

const UARTConfig VisionPort::UART_CONFIG = {
        nullptr,
        nullptr,
        VisionPort::uart_rx_callback,  // callback function when the buffer is filled
        VisionPort::uart_char_callback,
        VisionPort::uart_err_callback,
        115200, // speed
        0,
        0,
        0
};

void VisionPort::start(tprio_t thread_prio) {

    // Start UART driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    uartStartReceive(UART_DRIVER, FRAME_SOF_SIZE, &pak);

//    txThread.start(thread_prio);
}

void VisionPort::TXThread::main() {
    setName("Vision_TX");
    while (!shouldTerminate()) {

        size_t tx_pak_size = FRAME_HEADER_SIZE + CMD_ID_SIZE + sizeof(gimbal_info_t) + FRAME_TAIL_SIZE;

        tx_pak.header.sof = 0xA5;
        tx_pak.header.data_length = sizeof(gimbal_info_t);
        tx_pak.header.seq = tx_seq++;
        Append_CRC8_Check_Sum((uint8_t *) &tx_pak, FRAME_HEADER_SIZE);

        tx_pak.cmdID = GIMBAL_INFO_CMD_ID;
        tx_pak.gimbal_current_.yawAngle = GimbalSKD::get_accumulated_angle(GimbalSKD::YAW);
        tx_pak.gimbal_current_.pitchAngle = GimbalSKD::get_accumulated_angle(GimbalSKD::PITCH);
        // TODO: target velocities?
        tx_pak.gimbal_current_.yawVelocity = GimbalSKD::get_actual_velocity(GimbalSKD::YAW);
        tx_pak.gimbal_current_.pitchVelocity = GimbalSKD::get_actual_velocity(GimbalSKD::PITCH);
        Append_CRC16_Check_Sum((uint8_t *) &tx_pak, tx_pak_size);

        uartSendFullTimeout(UART_DRIVER, &tx_pak_size, &tx_pak, TIME_MS2I(7));
//    uartStartSend(UART_DRIVER, tx_pak_size, (uint8_t *) &tx_pak);  // it has some problem

        sleep(TIME_MS2I(TX_THREAD_INTERVAL));
    }
}

bool VisionPort::getControlCommand(VisionControlCommand &command) {
    if (WITHIN_RECENT_TIME(last_update_time, 2000)) {
        command = {vision_yaw_target, vision_pitch_target};
        return true;
    } else {
        return false;
    }
}


void VisionPort::uart_rx_callback(UARTDriver *uartp) {

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
                rx_status = WAIT_REMAINING_HEADER;
            } // else, keep waiting for SOF
            break;

        case WAIT_REMAINING_HEADER:

            if (Verify_CRC8_Check_Sum(pak_uint8, FRAME_HEADER_SIZE)) {
                rx_status = WAIT_CMD_ID_DATA_TAIL; // go to next status
            } else {
                rx_status = WAIT_STARTING_BYTE;
            }
            break;

        case WAIT_CMD_ID_DATA_TAIL:

            if (Verify_CRC16_Check_Sum(pak_uint8, FRAME_HEADER_SIZE + CMD_ID_SIZE + pak.header.data_length + FRAME_TAIL_SIZE)) {

                switch (pak.cmdID) {
                    case VISION_CONTROL_CMD_ID: {
                        // chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
                        {
                            if (last_update_time == 0 || !WITHIN_RECENT_TIME(last_update_time, 1000)) {
                                // No previous data or out-of-date, use the latest data
                                last_gimbal_yaw = GimbalSKD::get_accumulated_angle(GimbalBase::YAW);
                                last_gimbal_pitch = GimbalSKD::get_accumulated_angle(GimbalBase::PITCH);
                            }  // otherwise, use gimbal angles at last time

                            if (pak.vision.mode == ABSOLUTE_ANGLE) {
                                vision_yaw_target = pak.vision.yaw;
                                vision_pitch_target = pak.vision.pitch;
                            } else {
                                /*
                                 * Use gimbal angles at last control command, which is roughly the angles when the
                                 * image is captured.
                                 */
                                vision_yaw_target = pak.vision.yaw + last_gimbal_yaw;
                                vision_pitch_target = pak.vision.pitch + last_gimbal_pitch;
                            }

                            // Record current gimbal angles, which will be used for next control command
                            last_gimbal_yaw = GimbalSKD::get_accumulated_angle(GimbalBase::YAW);
                            last_gimbal_pitch = GimbalSKD::get_accumulated_angle(GimbalBase::PITCH);
                            last_update_time = SYSTIME;
                        }
                        // chSysUnlock();  /// --- EXIT S-Locked state ---
                    }
#ifdef VISION_PORT_DEBUG
                        LED::green_toggle();
#warning "VisionPort: in debug mode now"
#endif
                        break;
                }
            }

            rx_status = WAIT_STARTING_BYTE;

            break;
    }

    switch (rx_status) {
        case WAIT_STARTING_BYTE:
            uartStartReceiveI(uartp, FRAME_SOF_SIZE, pak_uint8);
            break;
        case WAIT_REMAINING_HEADER:
            uartStartReceiveI(uartp, FRAME_HEADER_SIZE - FRAME_SOF_SIZE, pak_uint8 + FRAME_SOF_SIZE);
            break;
        case WAIT_CMD_ID_DATA_TAIL:
            uartStartReceiveI(uartp, CMD_ID_SIZE + pak.header.data_length + FRAME_TAIL_SIZE,
                              pak_uint8 + FRAME_HEADER_SIZE);
            break;
    }

    chSysUnlockFromISR();  /// --- EXIT I-Locked state ---

}

void VisionPort::uart_err_callback(UARTDriver *uartp, uartflags_t e) {
    (void) uartp;
    (void) e;
#ifdef VISION_PORT_DEBUG
    for (unsigned i = 0; i < 8; i++) {
        if (e & (1U << i)) LED::led_toggle(i + 1);
    }
#warning "VisionPort: in debug mode now"
#endif
}

void VisionPort::uart_char_callback(UARTDriver *uartp, uint16_t c) {
    (void) uartp;
    (void) c;
}