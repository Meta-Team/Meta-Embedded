//
// Created by liuzikai on 2019-01-27.
//

// Header for vehicle. VEHICLE is set for each target in CMakeLists.txt.

#define INFANTRY_ONE 1
#define HERO 2
#define ENGINEER 3
#define INFANTRY_TWO 4
#define INFANTRY_THREE 5

#if VEHICLE == INFANTRY_ONE
#include "vehicle_infantry_one.h"
#elif VEHICLE == ENGINEER
#include "vehicle_engineer.h"
#endif

// Basic headers
#include "ch.hpp"
#include "hal.h"

// Debug headers
#include "led.h"
#include "serial_shell.h"

// Modules and basic communication channels
#include "can_interface.h"

// Interfaces
#include "gimbal_interface.h"
#include "mpu6500.h"

// Controllers
#include "gimbal_controller.h"

using namespace chibios_rt;


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority,
    // so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled,
    // vehicle_configs() should implement infinite loop
    while(true) {}
#else
    // When vehicle_configs() quits, the vehicle_configs thread will somehow
        // enter an infinite loop, so we set the priority to lowest
        // before quitting, to let other threads run normally
        BaseThread::setPriority(1);
#endif
    return 0;
}