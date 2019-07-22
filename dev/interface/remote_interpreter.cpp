//
// Created by liuzikai on 2018/8/6.
// Inspired by ChibiStudio template code.
//

/**
 * @file    remote_interpreter.cpp
 * @brief   Module to receive date from DR16 receiver and interpret data to specific formats
 *
 * @addtogroup remote
 * @{
 */

#include "remote_interpreter.h"

#include "common_macro.h"
#include "led.h"
#include "shell.h"

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

bool Remote::synchronizing = false;

#if REMOTE_USE_EVENTS
// See macro EVENTSOURCE_DECL() for initialization style
event_source_t Remote::s_change_event = {(event_listener_t *)(&Remote::s_change_event)};
event_source_t Remote::mouse_press_event = {(event_listener_t *)(&Remote::mouse_press_event)};
event_source_t Remote::mouse_release_event = {(event_listener_t *)(&Remote::mouse_release_event)};
event_source_t Remote::key_press_event = {(event_listener_t *)(&Remote::key_press_event)};
event_source_t Remote::key_release_event = {(event_listener_t *)(&Remote::key_release_event)};
#endif

char Remote::rx_buf_[Remote::RX_FRAME_SIZE];
UARTConfig Remote::REMOTE_UART_CONFIG = {
        nullptr,
        nullptr,
        uart_received_callback_, // callback function when the buffer is filled
        nullptr,
        nullptr,
        100000, // speed
        USART_CR1_PCE,
        0,
        0,
};

/**
 * @note DO NOT use printf, LOG, etc. in this function since it's an ISR callback.
 */
void Remote::uart_received_callback_(UARTDriver *uartp) {

    if (synchronizing) return;

    chSysLockFromISR();  /// --- ENTER I-Locked state. DO NOT use LOG, printf, non I-Class functions or return ---

    /// RC

    rc.ch0 = (((rx_buf_[0] | rx_buf_[1] << 8) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch1 = (((rx_buf_[1] >> 3 | rx_buf_[2] << 5) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch2 = (((rx_buf_[2] >> 6 | rx_buf_[3] << 2 | rx_buf_[4] << 10) & 0x07FF) - 1024.0f) / 660.0f;
    rc.ch3 = (((rx_buf_[4] >> 1 | rx_buf_[5] << 7) & 0x07FF) - 1024.0f) / 660.0f;

    auto s1 = (rc_status_t) ((rx_buf_[5] >> 6) & 0x0003);
    auto s2 = (rc_status_t) ((rx_buf_[5] >> 4) & 0x0003);

#if REMOTE_USE_EVENTS

    if (s1 != rc.s1 || s2 != rc.s2) {
        chEvtBroadcastI(&s_change_event);
    }

#endif

    rc.s1 = s1;
    rc.s2 = s2;

    rc.wheel = ((((int16_t) rx_buf_[16] | ((int16_t) rx_buf_[17] << 8)) & 0x07FF) - 1024.0f) / 660.0f;


    /// Mouse

    mouse.x = ((int16_t) (rx_buf_[6] | rx_buf_[7] << 8)) / 32767.0f;
    mouse.y = ((int16_t) (rx_buf_[8] | rx_buf_[9] << 8)) / 32767.0f;
    mouse.z = ((int16_t) (rx_buf_[10] | rx_buf_[11] << 8)) / 32767.0f;

    auto press_left = (bool) rx_buf_[12];
    auto press_right = (bool) rx_buf_[13];

#if REMOTE_USE_EVENTS

    /**
     * Similar to key_code below.
     */

    uint32_t new_mouse_code =       press_left << MOUSE_LEFT |       press_right << MOUSE_RIGHT;
    uint32_t old_mouse_code = mouse.press_left << MOUSE_LEFT | mouse.press_right << MOUSE_RIGHT;
    uint32_t diff_mouse_code = new_mouse_code ^ old_mouse_code;
    if (new_mouse_code & diff_mouse_code) chEvtBroadcastFlagsI(&mouse_press_event, new_mouse_code & diff_mouse_code);
    if (old_mouse_code & diff_mouse_code) chEvtBroadcastFlagsI(&mouse_release_event, old_mouse_code & diff_mouse_code);

#endif

    mouse.press_left = press_left;
    mouse.press_right = press_right;


    // Key

    uint32_t key_code_ = static_cast<uint16_t>(rx_buf_[14] | rx_buf_[15] << 8);

#if REMOTE_USE_EVENTS

    /**
     * Explanation of using bitwise operators to generate event flags:
     *
     * 1. diff_key_code                 - XOR marks the different bits as 1 into diff_key_code
     * 2. key_code_ & diff_key_code     - Unchanged bits are masked to 0. Released keys are also 0. 1 bits are those
     *                                    newly pressed.
     * 3. key.key_code_ & diff_key_code - Unchanged bits are masked to 0. Originally non-pressed keys (now changed to 1)
     *                                    are also 0. Originally pressed keys (now change to 0) are 1. So 1 bits are
     *                                    those newly released.
     * Notice that 2 and 3 are inverse of each other.
     *
     *
     * For example, W releases and S presses
     *
     * |               | W | S | A | D |
     * | key.key_code_ | 1 | 0 | 1 | 0 |
     * | key_code_     | 0 | 1 | 1 | 0 |
     * | diff_key_code | 1 | 1 | 0 | 0 |
     * | pressed flag  | 0 | 1 | 0 | 0 | (S key has pressed event)
     * | released flag | 1 | 0 | 0 | 0 | (W key has released event)
     */

    uint32_t diff_key_code = key_code_ ^ key.key_code_;
    if (    key_code_ & diff_key_code) chEvtBroadcastFlagsI(&key_press_event, key_code_ & diff_key_code);
    if (key.key_code_ & diff_key_code) chEvtBroadcastFlagsI(&key_release_event, key.key_code_ & diff_key_code);

#endif

    key.key_code_ = key_code_;


    // Final

    last_update_time = SYSTIME;

    // Restart the receive
    uartStartReceiveI(uartp, RX_FRAME_SIZE, rx_buf_);

    chSysUnlockFromISR();  /// --- EXIT S-Locked state ---
}

void Remote::uart_synchronize() {
    synchronizing = true;
    // Wait for no input in 5 ms, to avoid one receive starting from the middle of a frame
    while (true) {
        // For unknown reason, uartReceiveTimeout() seems not to wait for additional bytes if byte_received > 1
        size_t byte_received = 1;
        if (REMOTE_UART_DRIVER.rxstate != UART_RX_ACTIVE) {
            msg_t ret = uartReceiveTimeout(&REMOTE_UART_DRIVER, &byte_received, rx_buf_, TIME_MS2I(5));
            if (ret == MSG_TIMEOUT) break;
        } else {
            chThdSleepMicroseconds(20);  // sleep caller thread
        }
    }
    uartStartReceive(&REMOTE_UART_DRIVER, RX_FRAME_SIZE, rx_buf_);
    synchronizing = false;
}

void Remote::start() {
    uartStart(&REMOTE_UART_DRIVER, &REMOTE_UART_CONFIG);
    uart_synchronize();
}

Remote::key_t Remote::char2key(const char c) {
    for (unsigned i = 0; i < KEY_COUNT; i++) {
        if (c == KEY_CHAR_TABLE[i]) {
            return (key_t) i;
        }
    }
    return KEY_COUNT;
}

char Remote::key2char(const Remote::key_t key_index) {
    if (key_index >= KEY_COUNT) return '\0';
    return KEY_CHAR_TABLE[key_index];
}

/** @} */