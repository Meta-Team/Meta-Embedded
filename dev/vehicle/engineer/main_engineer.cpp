//
// Created by Kerui Zhu on 7/10/2019.
// Modified by LaiXinyi on 7/19/2019.
//

/// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "common_macro.h"

#include "shell.h"
#include "can_interface.h"
#include "ahrs.h"
#include "remote_interpreter.h"
#include "sd_card_interface.h"

#include "chassis_interface.h"
#include "engineer_elevator_interface.h"
#include "engineer_elevator_skd.h"
#include "engineer_grab_mech_interface.h"
#include "engineer_grab_skd.h"

#include "engineer_chassis_skd.h"
#include "engineer_chassis_logic.h"

#include "inspector_engineer.h"
#include "user_engineer.h"

#include "settings_engineer.h"

/// Vehicle Specific Configurations
#if defined(ENGINEER)
#include "vehicle_engineer.h"
#else
#error "File main_engineer.cpp should only be used for Engineer."
#endif

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard ahrs;

/// Local Constants
static const Matrix33 ON_BOARD_AHRS_MATRIX_ = ON_BOARD_AHRS_MATRIX;
static const Matrix33 GIMBAL_ANGLE_INSTALLATION_MATRIX_ = GIMBAL_ANGLE_INSTALLATION_MATRIX;
static const Matrix33 GIMBAL_GYRO_INSTALLATION_MATRIX_ = GIMBAL_GYRO_INSTALLATION_MATRIX;

PIDController::pid_params_t a2vPIDParameter[EngGrabMechBase::MOTOR_COUNT] = {SHOOT_PID_BULLET_LOADER_A2V_PARAMS,
                                                                             SHOOT_PID_BULLET_LOADER_A2V_PARAMS,
                                                                             SHOOT_PID_BULLET_LOADER_A2V_PARAMS};

PIDController::pid_params_t v2iPIDParameter[EngGrabMechBase::MOTOR_COUNT] = {{120, 2, 0, 12000, 12000},
                                                                             SHOOT_PID_BULLET_LOADER_V2I_PARAMS,
                                                                             SHOOT_PID_BULLET_LOADER_V2I_PARAMS};

EngGrabMechIF::motor_can_config_t CANCONFIG[3] = {  {MotorIFBase::can_channel_1, 3, CANInterface::M3508, CANInterface::M3508_WITH_DECELERATION_RATIO},
                                                    {MotorIFBase::can_channel_1, 2, CANInterface::M2006, CANInterface::M2006_WITH_DECELERATION_RATIO},
                                                    {MotorIFBase::can_channel_1, 4, CANInterface::M2006, CANInterface::M2006_WITH_DECELERATION_RATIO}};

EngineerGrabSKD::install_direction direct[3] = {EngineerGrabSKD::install_direction::POSITIVE,
                                                EngineerGrabSKD::install_direction::NEGATIVE,
                                                EngineerGrabSKD::install_direction::POSITIVE};

int main() {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    // Enable power
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);
    palSetPadMode(GPIOH, GPIOH_POWER2_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER2_CTRL);
    palClearPad(GPIOH, GPIOH_POWER4_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1
    InspectorE::init(&can1, &can2);
    LED::all_off();

    /// Setup Shell
    Shell::start(THREAD_SHELL_PRIO);
    Shell::addCommands(mainProgramCommands);
    chThdSleepMilliseconds(50);  // wait for logo to print :)

    BuzzerSKD::init(THREAD_BUZZER_SKD_PRIO);

    /// Setup SDCard
    if (SDCard::init()) {
        SDCard::read_all();
        LED::led_on(DEV_BOARD_LED_SD_CARD);  // LED 8 on if SD card inserted
    }

    LED::led_on(DEV_BOARD_LED_SYSTEM_INIT);  // LED 1 on now

    /// Setup On-Board AHRS
    ahrs.start(ON_BOARD_AHRS_MATRIX_, THREAD_MPU_PRIO, THREAD_IST_PRIO, THREAD_AHRS_PRIO);
    while(!ahrs.AHRS_ready()) {
        chThdSleepMilliseconds(5);
    }

    /// Setup CAN1 & CAN2
    can1.start(THREAD_CAN1_PRIO, THREAD_CAN1_SEND_PRIO);
    can2.start(THREAD_CAN2_PRIO, THREAD_CAN2_SEND_PRIO);
    chThdSleepMilliseconds(5);
    InspectorE::startup_check_can();  // check no persistent CAN Error. Block for 100 ms
    LED::led_on(DEV_BOARD_LED_CAN);  // LED 2 on now

    /// Setup Remote
    Remote::start();
    InspectorE::startup_check_remote();  // check Remote has signal. Block for 50 ms
    LED::led_on(DEV_BOARD_LED_REMOTE);  // LED 3 on now

    EngGrabMechIF::init(&can1, &can2, CANCONFIG);
    EngineerElevatorIF::init();
    LED::led_on(DEV_BOARD_LED_ELEVATOR);

    /// Setup ElevatorIF
    float init_angle;
    //EngineerElevatorSKD::start(THREAD_ELEVATOR_SKD_PRIO, 1.0f);
    ChassisIF::motor_can_config_t CHASSIS_MOTOR_CONFIG_[] = CHASSIS_MOTOR_CONFIG;
    /// Setup ChassisIF
    ChassisIF::init(&can1, &can2, CHASSIS_MOTOR_CONFIG_);
    chThdSleepMilliseconds(10);
    InspectorE::startup_check_chassis_feedback();  // check chassis motors has continuous feedback. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_CHASSIS);  // LED 6 on now

    /// Setup Referee
    Referee::init(THREAD_REFEREE_SENDING_PRIO);

    /// Complete Period 1
    LED::green_on();  // LED Green on now
    EngineerGrabSKD::start(THREAD_ROBOTIC_ARM_SKD_PRIO, direct);
    EngineerGrabSKD::load_a2v_pid_params(a2vPIDParameter);
    EngineerGrabSKD::load_v2i_pid_params(v2iPIDParameter);

    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Start Inspector and User Threads
    InspectorE::start_inspection(THREAD_INSPECTOR_PRIO, THREAD_INSPECTOR_REFEREE_PRIO);
    UserE::start(THREAD_USER_PRIO, THREAD_USER_ACTION_PRIO, THREAD_USER_CLIENT_DATA_SEND_PRIO);

    /// Complete Period 2
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel);  // Now play the startup sound

    //EngineerElevatorSKD::start(THREAD_ELEVATOR_SKD_PRIO, 1.0f);

    EngineerChassisSKD::start(CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE, THREAD_CHASSIS_SKD_PRIO);
    EngineerChassisSKD::load_pid_params(CHASSIS_PID_V2I_PARAMS);
    EngineerChassisLG::start(&ahrs, GIMBAL_ANGLE_INSTALLATION_MATRIX_, GIMBAL_GYRO_INSTALLATION_MATRIX_, CHASSIS_FOLLOW_PID_THETA2V_PARAMS, THREAD_CHASSIS_LG_PRIO);
    /*** ------------------------ Period 3. End of main thread ----------------------- ***/

    // Entering empty loop with low priority
#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}