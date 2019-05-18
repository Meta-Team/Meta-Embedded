//
// Created by liuzikai on 2019-01-27.
//

// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"

#include "can_interface.h"

#include "buzzer.h"
#include "remote_interpreter.h"
#include "elevator_interface.h"
#include "robotic_arm.h"
#include "chassis.h"

// Vehicle specific config
#if defined(ENGINEER) // specified in CMakeLists.txt
#include "vehicle_engineer.h"
#else
#error "main_engineer.cpp should only be used for Engineer."
#endif

// Board guard
#if defined(BOARD_RM_2018_A) // specified in build profile
#else
#error "Engineer is only developed for RM board 2018 A."
#endif


// Threads
#include "thread_chassis.hpp"
#include "thread_elevator.hpp"
#include "thread_action_trigger.hpp"

// State machines
#include "state_machine_stage_climb.h"
#include "state_machine_bullet_fetch.h"


static void cmd_elevator_set_target_position(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "e_set front_pos[cm] back_pos[cm] positive for VEHICLE to DOWN");
        return;
    }
    Elevator::calc_back(Shell::atof(argv[0]));
    Elevator::calc_front(Shell::atof(argv[1]));
    chprintf(chp, "Target pos = %f, %f" SHELL_NEWLINE_STR, Shell::atof(argv[0]), Shell::atof(argv[1]));
}

// Shell commands to control the elevator interface directly
ShellCommand elevatorInterfaceCommands[] = {
        {"e_set", cmd_elevator_set_target_position},
        {nullptr, nullptr}
};


// Interfaces
CANInterface can1(&CAND1);

// Threads
ChassisThread chassisThread;
ActionTriggerThread actionTriggerThread;

StageClimbThread elevatorThread;
RoboticArmThread roboticArmThread;

int main(void) {

    /*** --------------------------- Period 1. Basic Setup --------------------------- ***/

    /** Basic Initializations **/
    halInit();
    chibios_rt::System::init();
    LED::green_off();
    LED::red_off();

    /** Debug Setup **/
    Shell::start(HIGHPRIO);
    Shell::addCommands(elevatorInterfaceCommands);

    /** Basic IO Setup **/
    can1.start(HIGHPRIO - 1);
    Remote::start_receive();

    Chassis::init(&can1, CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE);
    Elevator::init(&can1);
    RoboticArm::init(&can1, ROBOTIC_ARM_INSIDE_ANGLE_RAW);

    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

//    while (palReadPad(STARTUP_BUTTON_PAD, STARTUP_BUTTON_PIN_ID) != STARTUP_BUTTON_PRESS_PAL_STATUS) {
//        // Wait for the button to be pressed
//        LED::green_toggle();
//        chThdSleepMilliseconds(300);
//    }
//    /** User has pressed the button **/

    LED::green_on();

//    RoboticArm::reset_front_angle();

    /** Echo Gimbal Raws and Converted Angles **/
    chThdSleepMilliseconds(500);
    LOG("RA Motor: %u, %f", RoboticArm::motor_last_actual_angle_raw, RoboticArm::get_motor_actual_angle());

    actionTriggerThread.start(NORMALPRIO - 2);

    /** Start Logic Control Thread **/
    chassisThread.start(NORMALPRIO + 2);

    /** Play the Startup Sound **/
    Buzzer::play_sound(Buzzer::sound_startup_intel, LOWPRIO);


    /*** ------------------------ Period 3. End of main thread ----------------------- ***/

    // Entering empty loop with low priority

#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(1);
#endif
    return 0;
}