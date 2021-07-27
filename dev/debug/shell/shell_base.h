/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    shell_base.h
 * @brief   Revised CLI shell header from ChibiOS Shell.
 *
 * @addtogroup shell_chibios
 * @{
 */

#ifndef SHELL_BASE_H
#define SHELL_BASE_H

#include "shellconf.h"

/**
 * @brief   Command handler function type.
 */
typedef bool (*shellcmd_t)(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * @brief   Custom command entry type.
 */
typedef struct {
    const char            *sc_name;           /**< @brief Command name.       */
    const char            *sc_arguments;      /**< @brief Command argument    */
    shellcmd_t            sc_function;        /**< @brief Command function.   */
    void                  *sc_arg;            /**< @brief Command function.   */
} ShellCommand;

/**
 * @brief   Shell history type.
 */
typedef struct {
    char                   *sh_buffer;        /**< @brief Buffer to store command
                                                 history.                   */
    const int              sh_size;           /**< @brief Shell history buffer
                                                 size.                      */
    int                    sh_beg;            /**< @brief Beginning command index
                                                 in buffer.                 */
    int                    sh_end;            /**< @brief Ending command index
                                                 in buffer.                 */
    int                    sh_cur;            /**< @brief Currently selected
                                                 command in buffer.         */
} ShellHistory;

/**
 * @brief   Shell descriptor type.
 */
typedef struct {
    BaseSequentialStream  *sc_channel;        /**< @brief I/O channel associated
                                                 to the shell.              */
    ShellCommand          *sc_commands;       /**< @brief Shell extra commands
                                                 table.                     */
    mutex_t               *sc_mutex;
#if (SHELL_USE_HISTORY == TRUE) || defined(__DOXYGEN__)
    char                  *sc_histbuf;        /**< @brief Shell command history
                                                 buffer.                    */
    int             sc_histsize;        /**< @brief Shell history buffer
                                                 size.                      */
#endif
#if (SHELL_USE_COMPLETION == TRUE) || defined(__DOXYGEN__)
    char                  **sc_completion;    /**< @brief Shell command completion
                                                 buffer.                    */
#endif
} ShellConfig;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Send escape codes to move cursor to the beginning of the line
 *
 * @param[in] stream    pointer to a @p BaseSequentialStream object
 *
 * @notapi
 */
#define _shell_reset_cur(stream) chprintf(stream, "\033[%dD\033[%dC",       \
                                          SHELL_MAX_LINE_LENGTH +           \
                                          strlen(SHELL_PROMPT_STR) + 2,     \
                                          strlen(SHELL_PROMPT_STR))

/**
 * @brief   Send escape codes to clear the rest of the line
 *
 * @param[in] stream    pointer to a @p BaseSequentialStream object
 *
 * @notapi
 */
#define _shell_clr_line(stream)   chprintf(stream, "\033[K")


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern event_source_t shell_terminated;
#endif

#ifdef __cplusplus
extern "C" {
#endif
void shellInit(void);
THD_FUNCTION(shellThread, p);
void shellExit(msg_t msg);
bool shellGetLine(ShellConfig *scfg, char *line,
                  unsigned size, ShellHistory *shp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* SHELL_BASE_H */

/** @} */
