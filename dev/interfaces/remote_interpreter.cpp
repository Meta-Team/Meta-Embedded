//
// Created by liuzikai on 2018/8/6.
// Inspired by ChibiStudio template code.
//

#include "remote_interpreter.h"

/**
 * Hardware configurations.
 *  RX: PB7
 *  Alt mode: 7 (UART1).
 *  Speed: 100000
 **/

static constexpr UARTConfig remoteUartConfig = {
        nullptr,
        nullptr,
        Remote::uart_received_callback, // callback function when the buffer is filled
        nullptr,
        nullptr,
        100000, // speed
        USART_CR1_PCE,
        0,
        0,
};

Remote::rc_t Remote::rc;
Remote::mouse_t Remote::mouse;
Remote::keyboard_t Remote::key;

char Remote::rx_buf[Remote::rx_buf_size];

/**
 * @brief callback function when the buffer is filled
 * @param uartp
 */
void Remote::uart_received_callback(UARTDriver *uartp) {
    (void) uartp;

    chSysLock();

    rc.ch0 = (((rx_buf[0] | rx_buf[1] << 8) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch1 = (((rx_buf[1] >> 3 | rx_buf[2] << 5) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch2 = (((rx_buf[2] >> 6 | rx_buf[3] << 2 | rx_buf[4] << 10) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch3 = (((rx_buf[4] >> 1 | rx_buf[5] << 7) & 0x07FF) - 1024.0f) / 660.0f;

    rc.s1 = (RemoteRCSStatus) ((rx_buf[5] >> 6) & 0x0003);
    rc.s2 = (RemoteRCSStatus) ((rx_buf[5] >> 4) & 0x0003);

    mouse.x = (int16_t) (rx_buf[6] | rx_buf[7] << 8);
    mouse.y = (int16_t) (rx_buf[8] | rx_buf[9] << 8);
    mouse.z = (int16_t) (rx_buf[10] | rx_buf[11] << 8);

    mouse.press_left = (bool) rx_buf[12];
    mouse.press_right = (bool) rx_buf[13];

    key._key_code = static_cast<uint16_t>(rx_buf[14] | rx_buf[15] << 8);

    chSysUnlock();

    uartStartReceive(uartp, REMOTE_DATA_BUF_SIZE, rx_buf);
}

/**
 * @brief Initialize a remote interpreter instance and return its pointer.
 * @return the pointer of remote interpreter.
 */
void Remote::start_receive() {
//    palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
//    palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));
    uartStart(&UARTD1, &remoteUartConfig);
    uartStartReceive(&UARTD1, rx_buf_size, rx_buf);
}