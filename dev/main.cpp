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
#include "buzzer.h"
#include "mpu6500.h"
#include "remote_interpreter.h"
#include "gimbal_interface.h"

// Controllers
#include "gimbal_controller.h"

/**
 * Mode Table:
 * ------------------------------
 * Left  Right  Mode
 * ------------------------------
 *  UP    *    Safe
 *  MID   UP   Remote - Gimbal
 *
 *  -Others-   Safe
 * ------------------------------
 */


/** Declarations **/

static void can1_callback(CANRxFrame *rxmsg);

CANInterface can1(&CAND1, can1_callback);

/** Feedback Allocator **/

static void can1_callback(CANRxFrame *rxmsg) {
    switch (rxmsg->SID) {
        case 0x205:
        case 0x206:
        case 0x207:
            GimbalInterface::process_motor_feedback(rxmsg);
            break;
        default:
            break;
    }
}

/** Threads **/

/**
 * @brief thread to update MPU6500 data
 */
class MPU6500Thread : public chibios_rt::BaseStaticThread<256> {
    static constexpr unsigned int update_interval = 25; // [ms]

    void main() final {
        setName("mpu6500");
        MPU6500Controller::start();
        while (!shouldTerminate()) {
            MPU6500Controller::getData();
            sleep(TIME_MS2I(update_interval));
        }
    }
} mpu6500Thread;

/**
 * @brief thread to control gimbal
 * @pre initialize GimbalInterface with CAN driver
 * @pre start the thread of updating MPU6500
 * @pre reset front angle properly
 */
class GimbalThread : public chibios_rt::BaseStaticThread<1024> {
    static constexpr unsigned int gimbal_thread_interval = 10; // [ms]

    void main() final {
        setName("gimbal");
        GimbalController::yaw.v_to_i_pid.change_parameters(GIMBAL_PID_YAW_V2I_KP,
                                                           GIMBAL_PID_YAW_V2I_KI,
                                                           GIMBAL_PID_YAW_V2I_KD,
                                                           GIMBAL_PID_YAW_V2I_I_LIMIT,
                                                           GIMBAL_PID_YAW_V2I_OUT_LIMIT);
        GimbalController::yaw.angle_to_v_pid.change_parameters(GIMBAL_PID_YAW_A2V_KP,
                                                               GIMBAL_PID_YAW_A2V_KI,
                                                               GIMBAL_PID_YAW_A2V_KD,
                                                               GIMBAL_PID_YAW_A2V_I_LIMIT,
                                                               GIMBAL_PID_YAW_A2V_OUT_LIMIT);
        GimbalController::pitch.v_to_i_pid.change_parameters(GIMBAL_PID_PITCH_V2I_KP,
                                                             GIMBAL_PID_PITCH_V2I_KI,
                                                             GIMBAL_PID_PITCH_V2I_KD,
                                                             GIMBAL_PID_PITCH_V2I_I_LIMIT,
                                                             GIMBAL_PID_PITCH_V2I_OUT_LIMIT);
        GimbalController::pitch.angle_to_v_pid.change_parameters(GIMBAL_PID_PITCH_A2V_KP,
                                                                 GIMBAL_PID_PITCH_A2V_KI,
                                                                 GIMBAL_PID_PITCH_A2V_KD,
                                                                 GIMBAL_PID_PITCH_A2V_I_LIMIT,
                                                                 GIMBAL_PID_PITCH_A2V_OUT_LIMIT);
        while (!shouldTerminate()) {
            if (Remote::rc.s1 == Remote::REMOTE_RC_S_MIDDLE && Remote::rc.s2 == Remote::REMOTE_RC_S_UP) {

                // Calculate target velocity
                float yaw_target_velocity = GimbalController::yaw.angle_to_v(GimbalInterface::yaw.actual_angle,
                                                                             Remote::rc.ch0 * 90);
                float pitch_target_velocity = GimbalController::pitch.angle_to_v(GimbalInterface::pitch.actual_angle,
                                                                                 Remote::rc.ch1 * 90);
                // Calculate target current
                GimbalInterface::yaw.target_current = (int) GimbalController::yaw.v_to_i(
                        GIMBAL_YAW_ACTUAL_VELOCITY, yaw_target_velocity);
                GimbalInterface::pitch.target_current = (int) GimbalController::pitch.v_to_i(
                        GIMBAL_PITCH_ACTUAL_VELOCITY, pitch_target_velocity);
            } else {
                GimbalInterface::yaw.target_current = GimbalInterface::pitch.target_current = 0;
            }

            GimbalInterface::send_gimbal_currents();

            sleep(TIME_MS2I(gimbal_thread_interval));
        }
    }
} gimbalThread;

int main(void) {

    // Basic initialization
    halInit();
    chibios_rt::System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);

    can1.start_can();
    can1.start_thread(HIGHPRIO - 1);

    GimbalInterface::start(&can1);

    mpu6500Thread.start(HIGHPRIO - 2);

    GimbalInterface::yaw.reset_front_angle();
    GimbalInterface::pitch.reset_front_angle();

    gimbalThread.start(NORMALPRIO);

    Buzzer::play_sound(Buzzer::sound_startup_intel, LOWPRIO);

#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle_configs() quits, the vehicle_configs thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}