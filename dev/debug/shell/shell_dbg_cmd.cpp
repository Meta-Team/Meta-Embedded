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
static void cmd_hello(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    if (argc > 0) {
        shellUsage(chp, "hello");
        return;
    }
    chprintf(chp, "Hello World from ChibiOS!" SHELL_NEWLINE_STR);
}

#define EMPTY_STACK_PATTERN 1431655765L //0x55555555

/**
 * Echo the CPU usage data and stack status of each threads.
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_show_thread_stats(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    if (argc > 0) {
        shellUsage(chp, "stats");
        return;
    }

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
    chprintf(chp, "        thread name       best    average      worst    count   all  free stack" SHELL_NEWLINE_STR);

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

        chprintf(chp, "%19s %10lu %10lu %10lu %8lu  %3u%%  %10lu" SHELL_NEWLINE_STR,
                 thd->name, thd->stats.best, (unsigned long)(thd->stats.cumulative / thd->stats.n),
                 thd->stats.worst, thd->stats.n, (unsigned int)(100 * thd->stats.cumulative / sum),
                 free_stack);

        thd_ref = reg.nextThread(thd_ref);
    }

}

/**
 * List of available shell commands.
 * @note Must be at the bottom of file, otherwise it won't find the functions.
 * @note {NULL, NULL} is a marker of end of list and mustn't be removed.
 */
ShellCommand shell_debug_commands[3] = {
    {"hello", cmd_hello},
    {"stats", cmd_show_thread_stats},
    {nullptr, nullptr}
};