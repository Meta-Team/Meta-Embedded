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
 * @file    chprintf.h
 * @brief   Revised mini printf-like functionality from ChibiOS Shell.
 *
 * @addtogroup shell_chibios
 * @{
 */

#ifndef PRINTF_H
#define PRINTF_H

#include <stdarg.h>

#if defined(SHELL_CONFIG_FILE)
#include "shellconf.h"
#endif

/**
 * @brief   Float type support.
 */
#if !defined(CHPRINTF_USE_FLOAT) || defined(__DOXYGEN__)
#define CHPRINTF_USE_FLOAT          FALSE
#endif

#ifdef __cplusplus
extern "C" {
#endif
int chvprintf(BaseSequentialStream *chp, const char *fmt, va_list ap);
int chprintf(BaseSequentialStream *chp, const char *fmt, ...);
int chsnprintf(char *str, size_t size, const char *fmt, ...);
#ifdef __cplusplus
}
#endif

#endif /* PRINTF_H */

/** @} */
