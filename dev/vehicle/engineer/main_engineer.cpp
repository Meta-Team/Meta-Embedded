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
#include "robotic_arm.h"

#include "chassis.h"
#include "elevator.h"

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


// Threads and State Machines
#include "thread_chassis.h"
#include "thread_elevator.h"
#include "thread_error_detect.hpp"
#include "state_machine_stage_climb.h"
#include "state_machine_bullet_fetch.h"
#include "thread_action_trigger.hpp"


// Interfaces
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

// Threads
ChassisThread chassisThread({CHASSIS_PID_V2I_PARAMS});
ElevatorThread elevatorThread({ELEVATOR_PID_A2V_PARAMS}, {ELEVATOR_PID_V2I_PARAMS});

StageClimbStateMachine stageClimbStateMachine(chassisThread, elevatorThread);
BulletFetchStateMachine bulletFetchStateMachine;

ActionTriggerThread actionTriggerThread(elevatorThread, bulletFetchStateMachine, stageClimbStateMachine);

ErrorDetectThread errorDetectThread;

static void cmd_elevator_set_target_position(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "e_set front_pos[cm] back_pos[cm] positive for VEHICLE to UP");
        return;
    }
    elevatorThread.set_front_target_height(Shell::atof(argv[0]));
    elevatorThread.set_back_target_height(Shell::atof(argv[1]));
    chprintf(chp, "Target pos = %f, %f" SHELL_NEWLINE_STR, Shell::atof(argv[0]), Shell::atof(argv[1]));
}

// Shell commands to control the elevator interface directly
ShellCommand elevatorInterfaceCommands[] = {
        {"e_set", cmd_elevator_set_target_position},
        {nullptr, nullptr}
};

int main(void) {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    LED::all_off();

    /** Setup Shell */
    Shell::start(HIGHPRIO);
    Shell::addCommands(elevatorInterfaceCommands);
    StateHandler::echoEvent(StateHandler::SHELL_START);
    // LED 1 on now

    /** Setup CAN1 & CAN2 */
    chThdSleepMilliseconds(5);
    startupCheckCAN();  // check no persistent CAN Error. Block for 100 ms
    StateHandler::echoEvent(StateHandler::CAN_START_SUCCESSFULLY);
    // LED 2 on now

    LED::led_on(3);
    // LED 3 on now

    /** Setup Remote */
    Remote::start_receive();
    startupCheckRemote();  // check Remote has signal. Block for 50 ms
    StateHandler::echoEvent(StateHandler::REMOTE_START_SUCCESSFULLY);
    // LED 4 on now

    /** Setup RoboticArm */
    RoboticArm::init(&can1);
    chThdSleepMilliseconds(10);
    startupCheckRoboticArmFeedback();  // check chassis motors has continuous feedback. Block for 50 ms
    StateHandler::echoEvent(StateHandler::ROBOTIC_ARM_CONNECT);
    // LED 5 on now

    /** Setup Chassis */
    Chassis::init(&can1, CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE);
    chThdSleepMilliseconds(10);
    startupCheckChassisFeedback();  // check chassis motors has continuous feedback. Block for 50 ms
    StateHandler::echoEvent(StateHandler::CHASSIS_CONNECTED);
    // LED 6 on now

    /** Setup Elevator */
    Elevator::init(&can1);
    chThdSleepMilliseconds(10);
    startupCheckElevatorFeedback();  // check chassis motors has continuous feedback. Block for 50 ms
    StateHandler::echoEvent(StateHandler::ELEVATOR_CONNECTED);
    // LED 7 on now

    StateHandler::echoEvent(StateHandler::MAIN_MODULES_SETUP_COMPLETE);
    // LED Green on now


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    RoboticArm::reset_front_angle();

    chassisThread.start(NORMALPRIO);
    elevatorThread.start(NORMALPRIO - 1);
    actionTriggerThread.start(NORMALPRIO - 2);
    errorDetectThread.start(LOWPRIO + 1);

    StateHandler::echoEvent(StateHandler::MAIN_THREAD_SETUP_COMPLETE);
    // Now play the startup sound

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