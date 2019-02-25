//
// Created by liuzikai on 2019-01-27.
//

// Header for vehicle. VEHICLE is set for each target in CMakeLists.txt.

// Basic headers (including board definitions, so they should be at the very beginning)
#include "ch.hpp"
#include "hal.h"

#if defined(ENGINEER) // defined in CMakeLists.txt.
#include "vehicle_engineer.h"
#else
#error "main_engineer.cpp should only be used for Infantry #1."
#endif

#if defined(BOARD_RM_2018_A) // defined in board.h (included in hal.h)
#define STARTUP_BUTTON_PAD GPIOB
#define STARTUP_BUTTON_PIN_ID GPIOB_USER_BUTTON
#define STARTUP_BUTTON_PRESS_PAL_STATUS PAL_HIGH
#else
#error "Engineer is only developed for RM board 2018 A."
#endif

// Debug headers
#include "led.h"
#include "serial_shell.h"

// Modules and basic communication channels
#include "can_interface.h"
#include "chassis_common.h"

// Interfaces
#include "buzzer.h"
#include "remote_interpreter.h"
#include "chassis_interface.h"
#include "elevator_interface.h"

// Controllers
#include "chassis_calculator.h"
#include "elevator_thread.h"

/**
 * Mode Table:
 * ------------------------------------------------------------
 * Left  Right  Mode
 * ------------------------------------------------------------
 *  UP    *     Safe
 *  MID  DOWN   Remote - Chassis + Elevator
 *  -Others-    Safe
 * ------------------------------------------------------------
 */


/** Declarations **/

CANInterface can1(&CAND1);

/** Threads **/

ElevatorThread elevatorThread;

/**
 * @name ChassisThread
 * @brief thread to control chassis
 * @pre RemoteInterpreter start receive
 * @pre initialize ChassisInterface with CAN driver
 */
class ChassisThread : public chibios_rt::BaseStaticThread<1024> {
    static constexpr unsigned int chassis_thread_interval = 20;

    void main() final {

        setName("chassis");

        ChassisController::change_pid_params(CHASSIS_PID_V2I_PARAMS);

        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::REMOTE_RC_S_MIDDLE && Remote::rc.s2 == Remote::REMOTE_RC_S_DOWN) {

                float target_vx;
                float target_vy;
                float target_w;

                if (elevatorThread.get_status() == elevatorThread.STOP) {
                    target_vx = -Remote::rc.ch2 * 1000.0f;
                    target_vy = -Remote::rc.ch3 * 1000.0f;
                    if (Remote::rc.s2 == Remote::REMOTE_RC_S_DOWN) {
                        target_w = Remote::rc.ch0 * 180.0f;
                    } else {
                        target_w = 0;
                    }
                } else {
                    target_vx = target_w = 0;
                    target_vy = elevatorThread.get_chassis_target_vy();
                }


                // Pack the actual velocity into an array
                float measured_velocity[4];
                for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                    measured_velocity[i] = ChassisInterface::motor[i].actual_angular_velocity;
                }

                // Perform calculation
                ChassisController::calc(measured_velocity, target_vx, target_vy, target_w);

                // Pass the target current to interface
                for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                    ChassisInterface::motor[i].target_current = (int) ChassisController::motor[i].target_current;
                }
            }

            ChassisInterface::send_chassis_currents();
            sleep(TIME_MS2I(chassis_thread_interval));

        }
    }
} chassisThread;

class ActionTriggerThread : public chibios_rt::BaseStaticThread<512> {
    static constexpr int action_trigger_thread_interval = 20; // [ms]

    systime_t start_time_pushing_ch1 = 0;
    bool has_started_buzzer = false;
    bool has_started_elevator = false;

    void main() final {
        setName("action_trigger");
        while (!shouldTerminate()) {
            if (Remote::rc.s1 == Remote::REMOTE_RC_S_MIDDLE && Remote::rc.s2 == Remote::REMOTE_RC_S_DOWN) {
                if (Remote::rc.ch1 > 0.8) {
                    if (start_time_pushing_ch1 == 0) {
                        start_time_pushing_ch1 = chVTGetSystemTime();
                    } else {
                        if (TIME_MS2I(chVTGetSystemTime() - start_time_pushing_ch1) > 2000 && !has_started_buzzer) {
                            Buzzer::play_sound(Buzzer::sound_alert, LOWPRIO);
                            has_started_buzzer = true;
                        }

                        if (TIME_MS2I(chVTGetSystemTime() - start_time_pushing_ch1) > 4000 && !has_started_elevator) {
                            elevatorThread.start_up_actions(NORMALPRIO + 1);
                            has_started_elevator = true;
                        }
                    }
                } else {
                    start_time_pushing_ch1 = 0;
                    has_started_buzzer = false;
                    has_started_elevator = false;
                }
            } else {
                start_time_pushing_ch1 = 0;
                has_started_buzzer = false;
                has_started_elevator = false;
            }
            sleep(TIME_MS2I(action_trigger_thread_interval));
        }
    }
} actionTriggerThread;

int main(void) {

    /*** --------------------------- Period 1. Basic Setup --------------------------- ***/

    /** Basic Initializations **/
    halInit();
    chibios_rt::System::init();
    LED::green_off();
    LED::red_off();

    /** Debug Setup **/
    Shell::start(HIGHPRIO);

    /** Basic IO Setup **/
    can1.start(HIGHPRIO - 1);
    Remote::start_receive();

    ChassisInterface::init(&can1);
    ElevatorInterface::init(&can1);

    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    while (palReadPad(STARTUP_BUTTON_PAD, STARTUP_BUTTON_PIN_ID) != STARTUP_BUTTON_PRESS_PAL_STATUS) {
        // Wait for the button to be pressed
        LED::green_toggle();
        chThdSleepMilliseconds(300);
    }

    /** User has pressed the button **/

    LED::green_on();

    actionTriggerThread.start(HIGHPRIO - 2);

    /** Start Logic Control Thread **/
    chassisThread.start(NORMALPRIO);

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