#ifndef META_INFANTRY_SERIAL_SHELL_H
#define META_INFANTRY_SERIAL_SHELL_H

#include "ch.hpp"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "shell_debug_commands.h"

#if defined(BOARD_RM_2018_A)
// RM_BOARD_2018_A: USART6_TX - PG14, USART6_RX - PG9
#elif defined(BOARD_RM_2017)
// RM_BOARD_2017: USART6_TX - PG14, USART6_RX - PG9
#else
#error "Buzzer interface has not been defined for selected board"
#endif

/**
 * @name Shell
 * @brief a serial shell interface using SD6 (UART6) as serial port.
 * @pre pins are configured properly in board.h, and driver are configured properly in halconf.h
 * @usage 1. start() with given thread priority
 *        2. use functions inside the interface
 */
class Shell {

private:

    /**
     * Container for the shell commands
     */
    static constexpr int maxCommandCount = 20;
    static ShellCommand shellCommands[];

    /**
     * Config of the serial port used by the shell.
     * First parameter is the bitrate.
     * Other three are for parity, stop bits, etc which we don't care.
     */
    static SerialConfig shellSerialConfig;

    /**
     * Config of the shell.
     * First parameter is the Serial port.
     * Second parameter is the list of commands, provided in serial_shell_commands.hpp
     */
    static ShellConfig shellConfig;

public:

    static bool enabled;  // whether the shell thread has started

    /**
     * @brief start the shell thread
     * @param prio priority
     * @return if the shell thread has already started return false
     */
    static bool start(tprio_t prio);

    /**
     * @brief add shell commands
     * @param command_list NULL-terminate command lsit
     * @return if one or more commands can't be added because the maximum command count has been reached, return false
     */
    static bool addCommands(ShellCommand *command_list);

    /**
     * @brief printf through shell
     * @param fmt
     * @param ...
     * @return the number of bytes that has been printed
     */
    static int printf(const char *fmt, ...);

    /**
     * @brief convert string to signed integer
     * @param s        the string to be converted
     * @return the integer
     * @note NO ERROR CHECK
     */
    static inline int atoi (const char* s) {
        int ret = 0;
        const char* p = s;
        if (*p == '-') p++;
        while (*p) {
            ret = ret * 10 + (*p - '0');
            p++;
        }
        if (s[0] == '-') ret = -ret;
        return ret;
    }

    /**
     * @brief convert string to float
     * @param s        the string to be converted
     * @return the float
     * @note NO ERROR CHECK
     */
    static inline float atof(const char* s){
        float rez = 0.0f;
        float fact = 1.0f;
        float sign = 1.0f;
        if (*s == '-'){
            s++;
            sign = -1.0f;
        }
        for (int point_seen = 0; *s; s++){
            if (*s == '.'){
                point_seen = 1;
                continue;
            };
            if (point_seen) {
                fact /= 10.0f;
                rez = rez + (float) (*s - '0') * fact;
            } else {
                rez = rez * 10.0f + (float) (*s - '0');
            }
        };
        return rez * sign;
    };
};


#endif