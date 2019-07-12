//
// Created by liuzikai on 2019-02-24.
// Modified by Zhu Kerui on 2019-7-12
//

#include "ch.hpp"
#include "hal.h"
#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "common_macro.h"
#include "buzzer.h"

#include "engineer_chassis_interface.h"
#include "engineer_chassis_skd.h"

CANInterface can1(&CAND1);

#if defined(BOARD_RM_2018_A)
#define STARTUP_BUTTON_PAD GPIOB
#define STARTUP_BUTTON_PIN_ID GPIOB_USER_BUTTON
#define STARTUP_BUTTON_PRESS_PAL_STATUS PAL_HIGH
#else
#error "Elevator thread is only developed for RM board 2018 A."
#endif


