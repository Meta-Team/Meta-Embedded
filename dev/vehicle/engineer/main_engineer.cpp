//
// Created by liuzikai on 2019-01-27.
//

// Basic headers (including board definitions, so they should be at the very beginning)
#include "ch.hpp"
#include "hal.h"

#if defined(ENGINEER) // defined in CMakeLists.txt.

#include "vehicle_engineer.h"

#else
#error "main_engineer.cpp should only be used for Engineer."
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

// Interfaces
#include "buzzer.h"
#include "remote_interpreter.h"
#include "elevator_interface.h"
#include "robotic_arm.h"

// Controllers
#include "chassis.h"
#include "elevator_thread.h"
#include "robotic_arm_thread.h"

/**
 * MODE TABLE:
 * ------------------------------------------------------------
 * Left  Right  Mode
 * ------------------------------------------------------------
 *  UP    *     Safe
 *  MID  DOWN   Remote - Chassis
 *  DOWN  *     PC - Chassis + Elevator + Robotic Arm
 *  -Others-    Safe
 * ------------------------------------------------------------
 * @note also update ONES doc
 */

static void cmd_elevator_set_target_position(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "e_set front_pos[cm] back_pos[cm] positive for VEHICLE to DOWN");
        return;
    }
    Elevator::apply_back_position(Shell::atof(argv[0]));
    Elevator::apply_front_position(Shell::atof(argv[1]));
    chprintf(chp, "Target pos = %f, %f" SHELL_NEWLINE_STR, Shell::atof(argv[0]), Shell::atof(argv[1]));
}

// Shell commands to control the elevator interface directly
ShellCommand elevatorInterfaceCommands[] = {
        {"e_set", cmd_elevator_set_target_position},
        {nullptr, nullptr}
};


/** Declarations **/

CANInterface can1(&CAND1);

/** Threads **/

ElevatorThread elevatorThread;
RoboticArmThread roboticArmThread;

/**
 * @name ChassisThread
 * @brief thread to control chassis
 * @pre RemoteInterpreter start receive
 * @pre initialize ChassisInterface with CAN driver
 */
class ChassisThread : public chibios_rt::BaseStaticThread<1024> {

    static constexpr unsigned int chassis_thread_interval = 20;

    /**
     * Params for user
     * @note also update ONES doc
     */
    static constexpr float PC_W_VY = -500.0f;
    static constexpr float PC_S_VY = 500.0f;
    static constexpr float PC_E_VX = -500.0f;
    static constexpr float PC_Q_VX = 500.0f;
    static constexpr float PC_A_W = -100.0f;
    static constexpr float PC_D_W = 100.0f;

    static constexpr float PC_CTRL_RATIO = 0.5f;

    void main() final {

        setName("chassis");

        Chassis::change_pid_params({CHASSIS_PID_V2I_PARAMS});

        while (!shouldTerminate()) {

            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) ||
                (Remote::rc.s1 == Remote::S_DOWN)) {

                float target_vx = 0, target_vy = 0, target_w = 0;

                if (elevatorThread.get_status() == elevatorThread.STOP) {  // if elevator thread is not in action,

                    // let user control the chassis
                    if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                        target_vx = -Remote::rc.ch2 * 1000.0f;
                        target_vy = -Remote::rc.ch3 * 1000.0f;
                        target_w = Remote::rc.ch0 * 180.0f;

                    } else if (Remote::rc.s1 == Remote::S_DOWN) {

                        if (Remote::key.w) target_vy = PC_W_VY;
                        else if (Remote::key.s) target_vy = PC_S_VY;
                        else target_vy = 0;

                        if (Remote::key.q) target_vx = PC_Q_VX;
                        else if (Remote::key.e) target_vx = PC_E_VX;
                        else target_vx = 0;

                        if (Remote::key.a) target_w = PC_A_W;
                        else if (Remote::key.d) target_w = PC_D_W;
                        else target_w = 0;

                        if (Remote::key.ctrl) {
                            target_vx *= PC_CTRL_RATIO;
                            target_vy *= PC_CTRL_RATIO;
                            target_w *= PC_CTRL_RATIO;
                        }
                    } else {
                        target_vx = target_vy = target_w = 0;
                    }

                } else {

                    target_vx = target_w = 0;
                    target_vy = elevatorThread.get_chassis_target_vy();

                }
                
                // Perform calculation
                Chassis::calc(target_vx, target_vy, target_w);
                
            } else {

                for (int i = 0; i < Chassis::MOTOR_COUNT; i++) {
                    Chassis::target_current[i] = 0;
                }

            }

            Chassis::send_chassis_currents();
            sleep(TIME_MS2I(chassis_thread_interval));

        }
    }
} chassisThread;


class ActionTriggerThread : public chibios_rt::BaseStaticThread<2048> {

    static constexpr int action_trigger_thread_interval = 20; // [ms]

    bool keyPressed = false;

    void main() final {

        setName("action_trigger");

        Elevator::change_pid_params({ELEVATOR_PID_A2V_PARAMS}, {ELEVATOR_PID_V2I_PARAMS});

        enum box_door_status_t {
            HIGH,
            LOW
        };

        box_door_status_t box_door_status = HIGH;
        palSetPad(GPIOH, GPIOH_POWER3_CTRL);

        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::S_DOWN) { // PC Mode

                if (Remote::key.v) {                                               // elevator lift up
                    if (!keyPressed) {
                        LOG_USER("press V");
                        Elevator::apply_back_position(-Elevator::STAGE_HEIGHT);
                        Elevator::apply_front_position(-Elevator::STAGE_HEIGHT);
                        keyPressed = true;
                    }
                } else if (Remote::key.b) {                                        // elevator lift down
                    if (!keyPressed) {
                        LOG_USER("press B");
                        Elevator::apply_back_position(0);
                        Elevator::apply_front_position(0);
                        keyPressed = true;
                    }
                } else if (Remote::key.z) {                                        // robotic arm initial outward
                    if (!keyPressed) {
                        LOG_USER("press Z");
                        keyPressed = true;
                    }
                    if (roboticArmThread.get_status() == roboticArmThread.STOP) {
                        if (!roboticArmThread.is_outward()) {
                            LOG("Trigger RA out");
                            roboticArmThread.start_initial_outward(NORMALPRIO - 3);
                        }
                    }
                } else if (Remote::key.x) {                                        // robotic arm fetch once
                    if (!keyPressed) {
                        LOG_USER("press X");
                        keyPressed = true;
                    }
                    if (roboticArmThread.get_status() == roboticArmThread.STOP) {
                        if (roboticArmThread.is_outward()) {
                            LOG("Trigger RA fetch");
                            roboticArmThread.start_one_fetch(NORMALPRIO - 3);
                        }
                    }
                } else if (Remote::key.c) {                                        // robotic arm final inward
                    if (!keyPressed) {
                        LOG_USER("press C");
                        keyPressed = true;
                    }
                    if (roboticArmThread.get_status() == roboticArmThread.STOP) {
                        if (roboticArmThread.is_outward()) {
                            LOG("Trigger RA in");
                            roboticArmThread.start_final_inward(NORMALPRIO - 3);
                        }
                    }
                } else if (Remote::key.g) {
                    if (!keyPressed) {
                        LOG_USER("press G");
                        keyPressed = true;
                    }
                    if (elevatorThread.get_status() == elevatorThread.STOP) {
                        elevatorThread.start_up_actions(NORMALPRIO - 2);
                    }
                } else if (Remote::key.f) {
                    if (!keyPressed) {
                        LOG_USER("press F");
                        keyPressed = true;
                        if (box_door_status == HIGH) {
                            LOG("Change to LOW");
                            box_door_status = LOW;
                            palClearPad(GPIOH, GPIOH_POWER3_CTRL);
                        }
                        else {
                            LOG("Change to HIGH");
                            box_door_status = HIGH;
                            palSetPad(GPIOH, GPIOH_POWER3_CTRL);
                        }
                    }
                }
                else {
                    keyPressed = false;
                }
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