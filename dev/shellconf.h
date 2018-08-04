#ifndef _SHELLCONF_H_
#define _SHELLCONF_H_

/*===========================================================================*/
/* ChibiOS Shell related settings.                                           */
/*===========================================================================*/

// Disable shell test commands, so we don't rely on ChibiOS's test codes
#define SHELL_CMD_TEST_ENABLED              FALSE

// Disable exiting the shell
#define SHELL_CMD_EXIT_ENABLED              FALSE

// Disable history in the shell
#define SHELL_USE_HISTORY                   FALSE

#endif