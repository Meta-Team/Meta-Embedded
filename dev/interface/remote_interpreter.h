//
// Created by liuzikai on 2018/8/6.
//

/**
 * This file contains Remote Interpreter
 * @brief Remote Interpreter is an interface between remote side and other
 *        components. It handles data flow from the DR16 receiver, and
 *        interprets data to specific format for other program components
 *        which need remote control data.
 */

#ifndef META_INFANTRY_REMOTE_INTERPRETER_H
#define META_INFANTRY_REMOTE_INTERPRETER_H

#include "ch.hpp"
#include "hal.h"

#include "serial_shell.h"

#if defined(BOARD_RM_2018_A)
// PB7 USART1_RX (alternate 7)
#define REMOTE_UART_PAD GPIOB
#define REMOTE_UART_PIN 7U
#define REMOTE_UART_ALTERNATE 7U
#define REMOTE_UART_DRIVER UARTD1
#elif defined(BOARD_RM_2017)
// PB7 USART1_RX (alternate 7)
#define REMOTE_UART_PAD GPIOB
#define REMOTE_UART_PIN 7U
#define REMOTE_UART_ALTERNATE 7U
#define REMOTE_UART_DRIVER UARTD1
#else
#error "Remote interpreter has not been defined for selected board"
#endif

#define REMOTE_ENABLE_USER_LOG FALSE

/**
 * @name Remote
 * @brief This class holds interpreted remote data.
 * @pre DBUS pin is configured properly in board.h
 */
class Remote {

public:

    enum rc_status_t {
        S_UP = 1,
        S_DOWN = 2,
        S_MIDDLE = 3
    };

    typedef struct {
        float ch0; // right horizontal, normalized: -1.0(leftmost) - 1.0(rightmost)
        float ch1; // right vertical, normalized: -1.0(downmost) - 1.0(upmost)
        float ch2; // left horizontal, normalized: -1.0(leftmost) - 1.0(rightmost)
        float ch3; // left vertical, normalized: -1.0(downmost) - 1.0(upmost)
        rc_status_t s1;
        rc_status_t s2;
    } rc_t;

    typedef struct {
        float x; // speed at x axis. Normalized: -1.0(fastest leftward) - 1.0(fastest rightward)
        float y; // speed at y axis. Normalized: -1.0(fastest upward) - 1.0(fastest downward)
        float z; // speed at z axis (unknown). Normalized: -1.0 - 1.0
        bool press_left;
        bool press_right;
    } mouse_t;

    typedef union {
        struct {
            bool w:1;
            bool s:1;
            bool a:1;
            bool d:1;
            bool shift:1;
            bool ctrl:1;
            bool q:1;
            bool e:1;
            bool r:1;
            bool f:1;
            bool g:1;
            bool z:1;
            bool x:1;
            bool c:1;
            bool v:1;
            bool b:1;
        };
        uint16_t _key_code; // hold key code data, for internal use
    } keyboard_t;

    /** Interface variables **/

    static rc_t rc;
    static mouse_t mouse;
    static keyboard_t key;

    /** Interface functions **/

    static void start_receive();

private:

    static void uart_received_callback_(UARTDriver *uartp); // call back function when data is completely retrieved

    static char rx_buf_[]; // store buf data retrieved from UART

    static const int RX_BUF_SIZE = 18;

    friend void uartStart(UARTDriver *uartp, const UARTConfig *config);
    friend void uartStartReceive(UARTDriver *uartp, size_t n, void *rxbuf);

    static constexpr UARTConfig REMOTE_UART_CONFIG = {
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

};

#endif //META_INFANTRY_REMOTE_INTERPRETER_H
