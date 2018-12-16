//
// Created by liuzikai on 2018/8/6.
//

#include "remote_interpreter.h"

// Initialize the object directly to avoid using `new` which may cause compile errors.
RemoteInterpreter _remote = RemoteInterpreter();

/**
 * Callback function when the buffer is filled
 * @param uartp
 */
void _remoteReceived(UARTDriver *uartp) {
    (void) uartp;
    chSysLock();
    _remote._processRemoteData();
    chSysUnlock();
    uartStartReceive(uartp, REMOTE_DATA_BUF_SIZE, _remote._rx_buf);
}

// Inspired by ChibiStudio template code
static constexpr UARTConfig remoteUartConfig = {
        NULL,
        NULL,
        _remoteReceived, // callback function when the buffer is filled
        NULL,
        NULL,
        100000, // speed
        USART_CR1_PCE,
        0,
        0,
};

/**
 * Initialize a remote interpreter instance and return its pointer.
 * @return the pointer of remote interpreter.
 */
RemoteInterpreter *remoteInit() {
    palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
    palSetPadMode(REMOTE_UART_PORT, REMOTE_UART_PAD, REMOTE_UART_MODE);
    uartStart(&REMOTE_UART_DRIVE, &remoteUartConfig);
    uartStartReceive(&REMOTE_UART_DRIVE, REMOTE_DATA_BUF_SIZE, _remote._rx_buf);
    return &_remote;
}

/**
 * Get the pointer initialized remote interpreter.
 * @return the pointer of remote interpreter.
 */
RemoteInterpreter *remoteGetInterpreter() {
    return &_remote;
}

// Interpret data from buf to specific formats
void RemoteInterpreter::_processRemoteData() {

    rc.ch0 = (((_rx_buf[0] | _rx_buf[1] << 8) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch1 = (((_rx_buf[1] >> 3 | _rx_buf[2] << 5) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch2 = (((_rx_buf[2] >> 6 | _rx_buf[3] << 2 | _rx_buf[4] << 10) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch3 = (((_rx_buf[4] >> 1 | _rx_buf[5] << 7) & 0x07FF) - 1024.0f) / 660.0f;

    rc.s1 = (RemoteRCSStatus) ((_rx_buf[5] >> 6) & 0x0003);
    rc.s2 = (RemoteRCSStatus) ((_rx_buf[5] >> 4) & 0x0003);

    mouse.x = (int16_t) (_rx_buf[6] | _rx_buf[7] << 8);
    mouse.y = (int16_t) (_rx_buf[8] | _rx_buf[9] << 8);
    mouse.z = (int16_t) (_rx_buf[10] | _rx_buf[11] << 8);

    mouse.press_left = (bool) _rx_buf[12];
    mouse.press_right = (bool) _rx_buf[13];

    key._key_code = _rx_buf[14] | _rx_buf[15] << 8;

}