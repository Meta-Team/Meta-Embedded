#include "shell_dbg_cmd.h"
using namespace chibios_rt;

/**
 * @file    shell_dbg_cmd.cpp
 * @brief   Our debug commands for shell.
 *
 * @addtogroup shell
 * @{
 */

/**
 * A demonstration of how to write a function for the shell.
 */
DEF_SHELL_CMD_START(cmd_hello)
    if (argc > 0) return false;
    Shell::printf("Hello World from ChibiOS!" ENDL);
    return true;
DEF_SHELL_CMD_END

#define EMPTY_STACK_PATTERN 1431655765L //0x55555555

/**
 * Echo the CPU usage data and stack status of each threads.
 * @param chp
 * @param argc
 * @param argv
 */
DEF_SHELL_CMD_START(cmd_show_thread_stats)
    if (argc > 0) return false;

    Registry reg; // a class managing registered threads
    ThreadReference thd_ref = reg.firstThread();
    thread_t *thd;

    // Iterate threads and sum up cumulative time.
    uint64_t sum = 0;
    while (!thd_ref.isNull()) {
        sum += thd_ref.getInner()->stats.cumulative;
        thd_ref = reg.nextThread(thd_ref);
    }

    // Print the header
    Shell::printf("        thread name       best    average      worst    count   all  free stack" ENDL);

    // Iterate threads again and show data.
    thd_ref = reg.firstThread();
    while (!thd_ref.isNull()) {
        thd = thd_ref.getInner();

        /*
         * Inspired by ChibiOS Eclipse plug-in.
         */
        uint32_t *stack_p = (uint32_t *)thd->ctx.sp;
        uint32_t *stklimit_p = (uint32_t *)thd->wabase;

        uint32_t *p = stklimit_p;
        /*
         * Empty stack area is filled with 0x55.
         * See chconf.h CH_DBG_FILL_THREADS.
         */
        while (p <= stack_p && *p == EMPTY_STACK_PATTERN) {
            p++;
        }

        unsigned long free_stack = (p - stklimit_p) * sizeof(uint32_t);

        Shell::printf("%-19s %10lu %10lu %10lu %8lu  %3u%%  %10lu" ENDL,
                 thd->name, thd->stats.best, (unsigned long)(thd->stats.cumulative / thd->stats.n),
                 thd->stats.worst, thd->stats.n, (unsigned int)(100 * thd->stats.cumulative / sum),
                 free_stack);

        thd_ref = reg.nextThread(thd_ref);
    }
    return true;
DEF_SHELL_CMD_END

/**
 * List of available shell commands.
 * @note Must be at the bottom of file, otherwise it won't find the functions.
 * @note {NULL, NULL} is a marker of end of list and mustn't be removed.
 */
Shell::Command shell_debug_commands[] = {
    {"hello", nullptr, cmd_hello, nullptr},
    {"stats", nullptr, cmd_show_thread_stats, nullptr},
    {nullptr, nullptr, nullptr, nullptr}
};