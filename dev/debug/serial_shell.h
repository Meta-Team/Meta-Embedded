//
// Created by Lan Tian on 2019-02-24.
//

/**
 * This file contain interface for serial shell and some macros for debug.
 */

#ifndef META_INFANTRY_SERIAL_SHELL_H
#define META_INFANTRY_SERIAL_SHELL_H

#include "ch.hpp"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "common_macro.h"

#include "shell_debug_commands.h"

#include "state_handler.h"

#if defined(BOARD_RM_2018_A)
// USART6_TX - PG14, USART6_RX - PG9
#elif defined(BOARD_RM_2017)
// USART6_TX - PG14, USART6_RX - PG9
#else
#error "Shell has not been defined for selected board"
#endif

/*** Debug Macro ***/

#define VA_ARGS(...) , ##__VA_ARGS__

#define LOG(fmt, ...) Shell::printf("[%u] " fmt SHELL_NEWLINE_STR, TIME_I2MS(chVTGetSystemTime()) VA_ARGS(__VA_ARGS__))
#define LOG_USER(fmt, ...) Shell::printf("[%u] USER " fmt SHELL_NEWLINE_STR, TIME_I2MS(chVTGetSystemTime()) VA_ARGS(__VA_ARGS__))
#define LOG_ERR(fmt, ...) Shell::printf("[%u] ERR " fmt SHELL_NEWLINE_STR, TIME_I2MS(chVTGetSystemTime()) VA_ARGS(__VA_ARGS__))
#define LOG_WARN(fmt, ...) Shell::printf("[%u] WARN " fmt SHELL_NEWLINE_STR, TIME_I2MS(chVTGetSystemTime()) VA_ARGS(__VA_ARGS__))
#define DBPRINTF(fmt, ...) Shell::printf("%s:%d:%s(): " fmt SHELL_NEWLINE_STR, __FILE__, __LINE__, __func__ VA_ARGS(__VA_ARGS__))


/**
 * @name Shell
 * @brief a serial shell interface using SD6 (UART6) as serial port.
 * @pre pins are configured properly in board.h, and driver are configured properly in halconf.h
 * @usage 1. Call start() with given thread priority
 *        2. Call addCommands() to add custom command to shell
 *           Call printf() and other helper function
 */
class Shell {

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
    static int atoi(const char *s);

    /**
     * @brief convert string to float
     * @param s        the string to be converted
     * @return the float
     * @note NO ERROR CHECK
     */
    static float atof(const char *s);

private:

    static ShellCommand shellCommands_[]; // Container for the shell commands

    static constexpr int MAX_COMMAND_COUNT = 20;

    /**
     * Config of the shell.
     * First parameter is the Serial port.
     * Second parameter is the list of commands, provided in serial_shell_commands.hpp
     * NOTICE: can't put it as a temporary variable in the start function
     */
    static ShellConfig shellConfig;

    /**
     * Config of the serial port used by the shell.
     * First parameter is the bitrate.
     * Other three are for parity, stop bits, etc which we don't care.
     */
    static constexpr SerialConfig SHELL_SERIAL_CONFIG = {115200,
                                                         0,
                                                         0,
                                                         0};

    static char complection_[MAX_COMMAND_COUNT][SHELL_MAX_LINE_LENGTH];

};


#endif