//
// Created by liuzikai on 2018/8/6.
//

/**
 * @file    remote_interpreter.h
 * @brief   Module to receive date from DR16 receiver and interpret data to specific formats
 *
 * @addtogroup remote
 * @{
 */

#ifndef META_INFANTRY_REMOTE_INTERPRETER_H
#define META_INFANTRY_REMOTE_INTERPRETER_H

#include "ch.hpp"
#include "hal.h"

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

// TODO: event functions are not tested yet
#define REMOTE_USE_EVENTS   TRUE

/**
 * @name Remote
 * @brief Module to receive date from DR16 receiver and interpret data to specific formats
 * @pre DBUS pins is configured properly in board.h
 * @note See DR16 document for components in this module
 * @usage 1. Invoke start()
 *        2. Use data in this module
 */
class Remote {

public:

    /**
     * Start remote interpreter
     */
    static void start();

    enum rc_status_t {
        S_UP = 1,
        S_DOWN = 2,
        S_MIDDLE = 3
    };

    typedef struct {
        float ch0;  // right horizontal, normalized: -1.0 (leftmost) - 1.0 (rightmost)
        float ch1;  // right vertical,   normalized: -1.0 (downmost) - 1.0 (upmost)
        float ch2;  // left horizontal,  normalized: -1.0 (leftmost) - 1.0 (rightmost)
        float ch3;  // left vertical,    normalized: -1.0 (downmost) - 1.0 (upmost)
        rc_status_t s1;
        rc_status_t s2;
        float wheel;  // scrolling wheel, normalized: -1.0 (upmost) - 1.0 (downmost)
    } rc_t;

    enum mouse_button_t {
        MOUSE_LEFT,
        MOUSE_RIGHT
    };

    typedef struct {
        float x;  // speed at x axis. Normalized: -1.0 (fastest leftward) - 1.0 (fastest rightward)
        float y;  // speed at y axis. Normalized: -1.0 (fastest upward)   - 1.0 (fastest downward)
        float z;  // speed at z axis. Normalized: -1.0 - 1.0 (unknown)
        bool press_left;
        bool press_right;
    } mouse_t;

    enum key_t {
        KEY_W,
        KEY_S,
        KEY_A,
        KEY_D,
        KEY_SHIFT,
        KEY_CTRL,
        KEY_Q,
        KEY_E,
        KEY_R,
        KEY_F,
        KEY_G,
        KEY_Z,
        KEY_X,
        KEY_C,
        KEY_V,
        KEY_B,
        KEY_COUNT
    };

    /**
     * Translate char to key index (key_t)
     * @param c   Char to be translated
     * @return Key index or KEY_COUNT if fails
     * @note KEY_SHIFT uses '^', KEY_CTRL uses '#'
     */
    static key_t char2key(const char c);

    /**
     * Translate key index (key_t) to char
     * @param key_index   Key index to be translated
     * @return Char corresponding to the key index, or '\0' if fails
     * @note KEY_SHIFT uses '^', KEY_CTRL uses '#'
     */
    static char key2char(const key_t key_index);

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
        uint16_t key_code_; // hold key code raw data, for internal use
    } keyboard_t;

    static rc_t rc;
    static mouse_t mouse;
    static keyboard_t key;

    static time_msecs_t last_update_time;

#if REMOTE_USE_EVENTS

    static event_source_t s_change_event;

    static event_source_t mouse_press_event;
    static event_source_t mouse_release_event;

    static event_source_t key_press_event;
    static event_source_t key_release_event;

#endif

    /**
     * Function to resynchronize UART receive to avoid receive starting from the middle of a frame
     * @note DO NOT call this function in lock state
     */
    static void uart_synchronize();

private:

    static constexpr const char* KEY_CHAR_TABLE = "WSAD^#QERFGZXCVB";

    /**
     * Call back function when a frame is completely retrieved
     * @param uartp   Pointer to UART driver
     * @note DO NOT use printf, LOG, etc. in this function since it's an ISR callback.
     */
    static void uart_received_callback_(UARTDriver *uartp);

    static bool synchronizing;

    static char rx_buf_[]; // store buf data retrieved from UART

    static const int RX_FRAME_SIZE = 18;

    friend void uartStart(UARTDriver *uartp, const UARTConfig *config);
    friend void uartStartReceive(UARTDriver *uartp, size_t n, void *rxbuf);
    friend class InspectorI;
    friend class InspectorH;
    friend class InspectorS;
    friend class InspectorA;
    friend class InspectorE;

    static UARTConfig REMOTE_UART_CONFIG;

};

#endif //META_INFANTRY_REMOTE_INTERPRETER_H

/** @} */