/**
 * @file    shellconf.h
 * @brief   Configuration for ChibiOS shell.
 *
 * @addtogroup config
 * @{
 */

#ifndef SHELLCONF_H
#define SHELLCONF_H

/*===========================================================================*/
/* ChibiOS Shell related settings.                                           */
/*===========================================================================*/

// Disable shell test commands, so we don't rely on ChibiOS's test codes
#define SHELL_CMD_TEST_ENABLED              FALSE

// Disable exiting the shell
#define SHELL_CMD_EXIT_ENABLED              FALSE

// Echo is printed back as typing, handled in shell_base.c, not controlled by printf mutex in Shell
#define SHELL_NO_ECHO_MODE                  TRUE
/// NOTE: this disable history and completion, regardless of the two options below

// Disable history in the shell
#define SHELL_USE_HISTORY                   TRUE

// Enable completion
#define SHELL_USE_COMPLETION                TRUE

// Clear shell prompt string
#define SHELL_PROMPT_STR                    ""

// Enable float in chprintf
#define CHPRINTF_USE_FLOAT                  TRUE

// Enlarge the maximum number of commands
#define SHELL_MAX_COMMAND_COUNT             40

// Enlarge the maximum number of arguments
#define SHELL_MAX_ARGUMENTS                 10

// Disable echo command as it's not so helpful (its default name also doesn't follow our standard)
#define SHELL_CMD_ECHO_ENABLED              FALSE

// String for new line
#define SHELL_NEWLINE_STR                   "\r\n"

#if !defined(BUILD_TARGET_NAME)
#define BUILD_TARGET_NAME "Unknown"
#endif

#define SHELL_WELCOME_STR                   SHELL_NEWLINE_STR SHELL_NEWLINE_STR \
                                            " ======== Welcome to Meta Shell ========" SHELL_NEWLINE_STR \
                                                                                       SHELL_NEWLINE_STR \
                                            "                /\\  /\\"                 SHELL_NEWLINE_STR \
                                            "               /  \\/  \\  /"             SHELL_NEWLINE_STR \
                                            "              /   /\\   \\/"              SHELL_NEWLINE_STR \
                                            "             /   /  \\  /\\"              SHELL_NEWLINE_STR \
                                            "                     \\/"                 SHELL_NEWLINE_STR \
                                            SHELL_NEWLINE_STR \
                                            "Current Program: " BUILD_TARGET_NAME      SHELL_NEWLINE_STR \
                                            SHELL_NEWLINE_STR \



#endif

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Shell History Constants
 */
#define SHELL_HIST_DIR_BK           0
#define SHELL_HIST_DIR_FW           1

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Do not echo input command
 * @author liuzikai
 */

#if !defined(SHELL_NO_ECHO_MODE)
#define SHELL_NO_ECHO_MODE                  FALSE
#endif

#if SHELL_NO_ECHO_MODE == TRUE

#if defined(SHELL_USE_HISTORY)
#undef SHELL_USE_HISTORY
#endif
#define SHELL_USE_HISTORY                   FALSE

#if defined(SHELL_USE_COMPLETION)
#undef SHELL_USE_COMPLETION
#endif
#define SHELL_USE_COMPLETION                FALSE

#if defined(SHELL_PROMPT_STR)
#undef SHELL_PROMPT_STR
#endif
#define SHELL_PROMPT_STR                    ""
#endif

/**
 * @brief   Shell maximum input line length.
 */
#if !defined(SHELL_MAX_LINE_LENGTH) || defined(__DOXYGEN__)
#define SHELL_MAX_LINE_LENGTH       128
#endif

/**
 * @brief   Shell maximum arguments per command.
 */
#if !defined(SHELL_MAX_ARGUMENTS) || defined(__DOXYGEN__)
#define SHELL_MAX_ARGUMENTS         4
#endif

/**
 * @brief   Shell maximum command history.
 */
#if !defined(SHELL_MAX_HIST_BUFF) || defined(__DOXYGEN__)
#define SHELL_MAX_HIST_BUFF         8 * SHELL_MAX_LINE_LENGTH
#endif

/**
 * @brief   Enable shell command history
 */
#if !defined(SHELL_USE_HISTORY) || defined(__DOXYGEN__)
#define SHELL_USE_HISTORY           FALSE
#endif

/**
 * @brief   Enable shell command completion
 */
#if !defined(SHELL_USE_COMPLETION) || defined(__DOXYGEN__)
#define SHELL_USE_COMPLETION        FALSE
#endif

/**
 * @brief   Shell Maximum Completions (Set to max commands with common prefix)
 */
#if !defined(SHELL_MAX_COMPLETIONS) || defined(__DOXYGEN__)
#define SHELL_MAX_COMPLETIONS       8
#endif

/**
 * @brief   Enable shell escape sequence processing
 */
#if !defined(SHELL_USE_ESC_SEQ) || defined(__DOXYGEN__)
#define SHELL_USE_ESC_SEQ           FALSE
#endif

/**
 * @brief   Prompt string
 */
#if !defined(SHELL_PROMPT_STR) || defined(__DOXYGEN__)
#define SHELL_PROMPT_STR            "ch> "
#endif

/**
 * @brief   Newline string
 */
#if !defined(SHELL_NEWLINE_STR) || defined(__DOXYGEN__)
#define SHELL_NEWLINE_STR            "\r\n"
#endif

/**
 * @brief   Welcome string
 */
#if !defined(SHELL_WELCOME_STR) || defined(__DOXYGEN__)
#define SHELL_WELCOME_STR            "> ChibiOS/RT Shell <"
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/** @} */