//
// Created by liuzikai on 2018/8/6.
//

/**
 * @component Remote Interpreter
 * @brief Remote Interpreter is an interface between remote side and other
 *        components. It handles data flow from the DR16 receiver, and
 *        interprets data to specific format for other program components
 *        which need remote control data.
 */

#ifndef META_INFANTRY_REMOTE_INTERPRETER_H
#define META_INFANTRY_REMOTE_INTERPRETER_H

#include "ch.hpp"
#include "hal.h"


#define REMOTE_DATA_BUF_SIZE 18

/**
 * @name Remote
 * @brief This class holds interpreted remote data.
 */
class Remote {

public:

    enum rc_status_t {
        REMOTE_RC_S_UP = 1,
        REMOTE_RC_S_DOWN = 2,
        REMOTE_RC_S_MIDDLE = 3
    };

    typedef struct {
        float ch0; // normalized: -1.0(leftmost) - 1.0(rightmost)
        float ch1; // normalized: -1.0(downmost) - 1.0(upmost)
        float ch2; // normalized: -1.0(leftmost) - 1.0(rightmost)
        float ch3; // normalized: -1.0(downmost) - 1.0(upmost)
        rc_status_t s1;
        rc_status_t s2;
    } rc_t;

    typedef struct {
        int x; // speed at x axis. Normalized: -1.0(fastest leftward) - 1.0(fastest rightward)
        int y; // speed at y axis. Normalized: -1.0(fastest upward) - 1.0(fastest downward)
        int z; // speed at z axis (unknown). Normalized: -1.0 - 1.0
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

    static void uart_received_callback(UARTDriver *uartp); // call back function when data is completely retrieved

private:

    static char rx_buf[]; // store buf data retrieved from UART

    static const int rx_buf_size = 18;

    friend void uartStart(UARTDriver *uartp, const UARTConfig *config);
    friend void uartStartReceive(UARTDriver *uartp, size_t n, void *rxbuf);

};

#endif //META_INFANTRY_REMOTE_INTERPRETER_H
