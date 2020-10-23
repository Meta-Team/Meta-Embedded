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

VisionPort::package_t VisionPort::pak;
VisionPort::rx_status_t VisionPort::rx_status;
VisionPort::enemy_info_t VisionPort::enemy_info;
time_msecs_t VisionPort::last_update_time = 0;
uint16_t VisionPort::tx_seq = 0;

const UARTConfig VisionPort::UART_CONFIG = {
        nullptr,
        nullptr,
        VisionPort::uart_rx_callback,  // callback function when the buffer is filled
        VisionPort::uart_char_callback,
        VisionPort::uart_err_callback,
        230400, // speed
        0,
        0,
        0
};

void VisionPort::init() {

    // Start UART driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    uartStartReceive(UART_DRIVER, FRAME_SOF_SIZE, &pak);
}

void VisionPort::send_gimbal(float yaw, float pitch) {

    package_t tx_pak;

    size_t tx_pak_size = FRAME_HEADER_SIZE + CMD_ID_SIZE + sizeof(gimbal_current_t) + FRAME_TAIL_SIZE;

    tx_pak.header.sof = 0xA5;
    tx_pak.header.data_length = sizeof(gimbal_current_t);
    tx_pak.header.seq = tx_seq++;
    Append_CRC8_Check_Sum((uint8_t *) &tx_pak, FRAME_HEADER_SIZE);

    tx_pak.cmd_id = 0xFF00;
    tx_pak.gimbal_current_.yaw = yaw;
    tx_pak.gimbal_current_.pitch = pitch;
    Append_CRC16_Check_Sum((uint8_t *) &tx_pak, tx_pak_size);

    uartSendFullTimeout(UART_DRIVER, &tx_pak_size, &tx_pak, TIME_MS2I(7));
//    uartStartSend(UART_DRIVER, tx_pak_size, (uint8_t *) &tx_pak);  // it has some problem
}

void VisionPort::send_enemy_color(bool is_blue) {
    package_t tx_pak;

    size_t tx_pak_size = FRAME_HEADER_SIZE + CMD_ID_SIZE + sizeof(enemy_color_t) + FRAME_TAIL_SIZE;

    tx_pak.header.sof = 0xA5;
    tx_pak.header.data_length = sizeof(enemy_color_t);
    tx_pak.header.seq = tx_seq++;
    Append_CRC8_Check_Sum((uint8_t *) &tx_pak, FRAME_HEADER_SIZE);

    tx_pak.cmd_id = 0xFF02;
    tx_pak.enemy_color_.is_blue = is_blue;
    Append_CRC16_Check_Sum((uint8_t *) &tx_pak, tx_pak_size);

    uartSendFullTimeout(UART_DRIVER, &tx_pak_size, &tx_pak, TIME_MS2I(10));
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

            if (Verify_CRC16_Check_Sum(pak_uint8,
                                       FRAME_HEADER_SIZE + CMD_ID_SIZE + pak.header.data_length + FRAME_TAIL_SIZE)) {

                switch (pak.cmd_id) {
                    case 0xFF01:
                        enemy_info = pak.enemy_info_;
                        last_update_time = SYSTIME;
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

    chSysUnlockFromISR();  /// --- EXIT S-Locked state ---

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