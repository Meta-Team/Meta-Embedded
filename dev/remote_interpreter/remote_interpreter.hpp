//
// Created by liuzikai on 2018/8/6.
//

/**
 * @component Remote Interpreter
 * @brief Remote Interpreter is an interface between remote side and other
 *        components. It handles data flow from the DR16 receiver, and
 *        interprets data to specific format for other program components
 *        which need remote control data.
 * @usage For a component using remote data, it's expected to hold a pointer
 *        of `RemoteInterpreter`, and retrieve data from it.
 *        For the main handler, it's expected to initialize an interpreter
 *        using `remoteInit()`, and pass the pointer to other components.
 */

#ifndef INSOULED_CHIBIOS_CPP_REMOTE_INTERPRETER_HPP
#define INSOULED_CHIBIOS_CPP_REMOTE_INTERPRETER_HPP

#include "ch.hpp"
#include "hal.h"

/** Hardware configurations. */
// Currently the rx pin of the receiver is PB7, with alternate mode 7 (UART1).
#define REMOTE_UART_DRIVE UARTD1
#define REMOTE_UART_PORT GPIOB
#define REMOTE_UART_PAD 7
#define REMOTE_UART_MODE PAL_MODE_ALTERNATE(7)
#define REMOTE_DATA_BUF_SIZE 18

/**
 * @name RemoteRCSStatus
 * @brief The status of the s1 and s2 control on rc.
 */
enum RemoteRCSStatus {
    REMOTE_RC_S_UP = 1,
    REMOTE_RC_S_DOWN = 2,
    REMOTE_RC_S_MIDDLE = 3
};

/**
 * @name RemoteInterpreter
 * @brief This class holds interpreted remote data.
 */
class RemoteInterpreter {
public:

    /**
     * @name rc
     * @brief data of remote controller.
     */
    struct {
        float ch0; // normalized: -1.0(leftmost) - 1.0(rightmost)
        float ch1; // normalized: -1.0(downmost) - 1.0(upmost)
        float ch2; // normalized: -1.0(leftmost) - 1.0(rightmost)
        float ch3; // normalized: -1.0(downmost) - 1.0(upmost)
        RemoteRCSStatus s1;
        RemoteRCSStatus s2;
    } rc;

    /**
     * @name mouse
     * @brief data of mouse.
     */
    struct {
        float x; // speed at x axis. Normalized: -1.0(fastest leftward) - 1.0(fastest rightward)
        float y; // speed at y axis. Normalized: -1.0(fastest upward) - 1.0(fastest downward)
        float z; // speed at z axis (unknown). Normalized: -1.0 - 1.0
        bool press_left;
        bool press_right;
    } mouse;

    /**
     * @name key
     * @brief status of keys.
     */
    union {
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
        uint16_t _key_code; // hold key code data, for internal use.
    } key;


    // Store buf data retrieved from UART.
    uint8_t _rx_buf[REMOTE_DATA_BUF_SIZE];

    // Call back function when data is completely retrieved.
    void _processRemoteData();

};

/**
 * Initialize a remote interpreter instance and return its pointer.
 * @return the pointer of remote interpreter.
 */
RemoteInterpreter *remoteInit();

/**
 * Get the pointer initialized remote interpreter.
 * @return the pointer of remote interpreter.
 */
RemoteInterpreter *remoteGetInterpreter();

#endif //INSOULED_CHIBIOS_CPP_REMOTE_INTERPRETER_HPP
