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

VisionPort::frame_header_t VisionPort::frame_header;
uint16_t VisionPort::cmd_id;
VisionPort::rx_status_t VisionPort::rx_status;
uint8_t VisionPort::rx_buf[RX_BUF_SIZE];
VisionPort::enemy_information_data_t VisionPort::enemy_info;

const UARTConfig VisionPort::UART_CONFIG = {
        nullptr,
        nullptr,
        VisionPort::uart_rx_callback, // callback function when the buffer is filled
        nullptr,
        nullptr,
        115200, // speed
        0,
        0,
        0,
};

void VisionPort::init() {
    // Start uart driver
    uartStart(UART_DRIVER, &UART_CONFIG);

    // Wait for starting byte
    rx_status = WAIT_STARTING_BYTE;
    uartStartReceive(UART_DRIVER, FRAME_SOF_SIZE, rx_buf);
}

void VisionPort::uart_rx_callback(UARTDriver *uartp) {
    (void) uartp;

    chSysLockFromISR();
    // Handle received data and transfer status properly
    switch (rx_status) {

        case WAIT_STARTING_BYTE:
            LED::green_toggle();
            if (rx_buf[0] == 0xA5) {
                rx_status = WAIT_REMAINING_HEADER;
            } // else, keep waiting for SOF
            Shell::printfI("[VisionPort] %x" SHELL_NEWLINE_STR, (unsigned) rx_buf[0]);
            break;

        case WAIT_REMAINING_HEADER:

            if (Verify_CRC8_Check_Sum((uint8_t *) rx_buf, FRAME_HEADER_SIZE)) {
                memcpy(&frame_header, rx_buf, FRAME_HEADER_SIZE);
                memcpy(&cmd_id, rx_buf + FRAME_HEADER_SIZE, CMD_ID_SIZE);
                rx_status = WAIT_CMD_ID_DATA_TAIL; // go to next status
            } else {
                Shell::printfI("[VisionPort] Invalid frameHeader!" SHELL_NEWLINE_STR);
                rx_status = WAIT_STARTING_BYTE;
            }
            break;

        case WAIT_CMD_ID_DATA_TAIL:

            if (Verify_CRC16_Check_Sum((uint8_t *) &rx_buf,
                                       FRAME_HEADER_SIZE + CMD_ID_SIZE + frame_header.data_length + FRAME_TAIL_SIZE)) {

                switch (cmd_id) {
                    case 0x0201:
                        memcpy(&enemy_info, rx_buf + FRAME_HEADER_SIZE + CMD_ID_SIZE, sizeof(enemy_information_data_t));
                        break;
                    default:
                        // FIXME: temporarily disabled since not all ID has been implemented
                        // LOG_ERR("[VisionPort] Unknown cmd_id %u", cmd_id);
                        break;
                }
            } else {
                Shell::printfI("[VisionPort] Invalid data of type %u!" SHELL_NEWLINE_STR, cmd_id);
            }

            rx_status = WAIT_STARTING_BYTE;

            break;
    }

    switch (rx_status) {
        case WAIT_STARTING_BYTE:
            uartStartReceiveI(uartp, FRAME_SOF_SIZE, rx_buf);
            break;
        case WAIT_REMAINING_HEADER:
            uartStartReceiveI(uartp, FRAME_HEADER_SIZE - FRAME_SOF_SIZE, rx_buf + FRAME_SOF_SIZE);
            break;
        case WAIT_CMD_ID_DATA_TAIL:
            uartStartReceiveI(uartp, CMD_ID_SIZE + frame_header.data_length + FRAME_TAIL_SIZE, rx_buf + FRAME_HEADER_SIZE);
            break;
    }

    chSysUnlockFromISR();

}