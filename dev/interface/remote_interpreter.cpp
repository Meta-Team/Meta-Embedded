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


Remote::rc_t Remote::rc;
Remote::mouse_t Remote::mouse;
Remote::keyboard_t Remote::key;
time_msecs_t Remote::last_update_time;
char Remote::rx_buf_[Remote::RX_FRAME_SIZE];
constexpr UARTConfig Remote::REMOTE_UART_CONFIG;

/**
 * Helper macro to log user behavior
 */
#define LOG_PRESS_AND_RELEASE(old_val, new_val, name) { \
    if (old_val != new_val) { \
        if (new_val) Shell::printfI("[REMOTE] press %s", name); \
        else Shell::printfI("[REMOTE] release %s", name); \
    } \
}

void Remote::uart_received_callback_(UARTDriver *uartp) {

    chSysLockFromISR();  /// ---------------------------------- Enter Critical Zone ----------------------------------

    rc.ch0 = (((rx_buf_[0] | rx_buf_[1] << 8) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch1 = (((rx_buf_[1] >> 3 | rx_buf_[2] << 5) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch2 = (((rx_buf_[2] >> 6 | rx_buf_[3] << 2 | rx_buf_[4] << 10) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch3 = (((rx_buf_[4] >> 1 | rx_buf_[5] << 7) & 0x07FF) - 1024.0f) / 660.0f;

#if REMOTE_USE_EVENTS

    rc_t rc_;
    rc_.s1 = (rc_status_t) ((rx_buf_[5] >> 6) & 0x0003);
    rc_.s2 = (rc_status_t) ((rx_buf_[5] >> 4) & 0x0003);
    if (rc_.s1 != rc.s1) LOG_USER("switch s1 from %d to %d", rc.s1, rc_.s1);
    if (rc_.s2 != rc.s2) LOG_USER("switch s2 from %d to %d", rc.s2, rc_.s2);
    rc = rc_;

    mouse_t mouse_;
    mouse_.press_left = (bool) rx_buf_[12];
    mouse_.press_right = (bool) rx_buf_[13];
    LOG_PRESS_AND_RELEASE(mouse.press_left, mouse_.press_left, "left");
    LOG_PRESS_AND_RELEASE(mouse.press_right, mouse_.press_right, "right");
    mouse = mouse_;

    keyboard_t key_;
    key_.key_code_ = static_cast<uint16_t>(rx_buf_[14] | rx_buf_[15] << 8);
    LOG_PRESS_AND_RELEASE(key.w, key_.w, "W");
    LOG_PRESS_AND_RELEASE(key.s, key_.s, "S");
    LOG_PRESS_AND_RELEASE(key.a, key_.a, "A");
    LOG_PRESS_AND_RELEASE(key.d, key_.d, "D");
    LOG_PRESS_AND_RELEASE(key.shift, key_.shift, "Shift");
    LOG_PRESS_AND_RELEASE(key.ctrl, key_.ctrl, "Ctrl");
    LOG_PRESS_AND_RELEASE(key.q, key_.q, "Q");
    LOG_PRESS_AND_RELEASE(key.e, key_.e, "E");
    LOG_PRESS_AND_RELEASE(key.r, key_.r, "R");
    LOG_PRESS_AND_RELEASE(key.f, key_.f, "f");
    LOG_PRESS_AND_RELEASE(key.g, key_.g, "G");
    LOG_PRESS_AND_RELEASE(key.z, key_.z, "Z");
    LOG_PRESS_AND_RELEASE(key.x, key_.x, "X");
    LOG_PRESS_AND_RELEASE(key.c, key_.c, "C");
    LOG_PRESS_AND_RELEASE(key.v, key_.v, "V");
    LOG_PRESS_AND_RELEASE(key.b, key_.b, "B");
    key = key_;

#else

    rc.s1 = (rc_status_t) ((rx_buf_[5] >> 6) & 0x0003);
    rc.s2 = (rc_status_t) ((rx_buf_[5] >> 4) & 0x0003);

    mouse.x = ((int16_t) (rx_buf_[6] | rx_buf_[7] << 8)) / 32767.0f;
    mouse.y = ((int16_t) (rx_buf_[8] | rx_buf_[9] << 8)) / 32767.0f;
    mouse.z = ((int16_t) (rx_buf_[10] | rx_buf_[11] << 8)) / 32767.0f;

    mouse.press_left = (bool) rx_buf_[12];
    mouse.press_right = (bool) rx_buf_[13];

    key.key_code_ = static_cast<uint16_t>(rx_buf_[14] | rx_buf_[15] << 8);

#endif

    last_update_time = SYSTIME;

    // Restart the receive
    uartStartReceive(uartp, RX_FRAME_SIZE, rx_buf_);

    chSysUnlockFromISR();  /// ---------------------------------- Exit Critical Zone ----------------------------------
}

void Remote::start() {
    uartStart(&REMOTE_UART_DRIVER, &REMOTE_UART_CONFIG);
    uartStartReceive(&REMOTE_UART_DRIVER, RX_FRAME_SIZE, rx_buf_);
}