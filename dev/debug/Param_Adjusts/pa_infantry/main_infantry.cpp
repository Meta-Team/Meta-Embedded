//
// Created by liuzikai on 2019-01-27.
//

/// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer.h"
#include "common_macro.h"

#include "shell.h"
#include "can_interface.h"
#include "ahrs.h"
#include "remote_interpreter.h"
#include "sd_card_interface.h"
#include "vision_port.h"
#include "super_capacitor_port.h"

#include "gimbal_interface.h"
#include "gimbal_scheduler.h"
#include "shoot_scheduler.h"
#include "gimbal_logic.h"
#include "shoot_logic.h"

#include "chassis_interface.h"
#include "chassis_scheduler.h"
#include "chassis_logic.h"

#include "InspectorI.h"
#include "user_infantry.h"

#include "settings_infantry.h"

/// Vehicle Specific Configurations
#if defined(PA_INFANTRY_THREE)                                                 /** Infantry #3 **/
#include "vehicle_infantry_three.h"
#elif defined(PA_INFANTRY_FOUR)                                                /** Infantry #4 **/
#include "vehicle_infantry_four.h"
#elif defined(PA_INFANTRY_FIVE)                                                /** Infantry #5 **/
#include "vehicle_infantry_five.h"
#else
#error "File main_infantry.cpp should only be used for Infantry #3, #4, #5."
#endif

/// Board Guard
#if defined(BOARD_RM_2018_A)
#else
#error "Infantry supports only RM Board 2018 A currently"
#endif

/// Instances
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard ahrs;

/// Local Constants
static const Matrix33 ON_BOARD_AHRS_MATRIX_ = ON_BOARD_AHRS_MATRIX;
static const Matrix33 GIMBAL_ANGLE_INSTALLATION_MATRIX_ = GIMBAL_ANGLE_INSTALLATION_MATRIX;
static const Matrix33 GIMBAL_GYRO_INSTALLATION_MATRIX_ = GIMBAL_GYRO_INSTALLATION_MATRIX;

/**-------------------------------------Param_adjust_Shell_Commands------------------------------------------*/
void UserI::set_mode(UserI::param_mode_t mode) {
    Param_Adjust_Mode = mode;
}

void UserI::set_gimbal_mode(UserI::gimbal_mode_t mode) {
    Gimbal_Mode = mode;
}

static void cmd_gimbal_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1')) {
        shellUsage(chp, "g_enable yaw(0/1) pitch(0/1)");
        return;
    }
    if(*argv[0]-'0' || *argv[1]-'0'){
        UserI::set_mode(UserI::GIMBAL);
    }
}

static void cmd_gimbal_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1')) {
        shellUsage(chp, "g_enable_fb yaw(0/1) pitch(0/1)");
        return;
    }
    if(*argv[0]-'0') {
        UserI::set_gimbal_mode(UserI::YAW);
    }else if(*argv[1]-'0'){
        UserI::set_gimbal_mode(UserI::PITCH);
    } else {
        UserI::set_gimbal_mode(UserI::YAW); // When 1 1 or 0 0, set feedback to original (YAW). TODO: Set a New mode that make feedback Thread print nothing... IS that works?
    }
}

static void cmd_gimbal_fix_front_angle(BaseSequentialStream *chp, int argc, char *argv[]){
    shellUsage(chp, "There's No gimbal fix front angle anymore.");
    return;
    // as this version of param adjust program used infantry and hero program, we may dont need a gimbal fix_front_angle program.
}

static void cmd_gimbal_set_target_velocities(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_v yaw_velocity pitch_velocity");
        return;
    }
    float target_v[2];
    target_v[0] = Shell::atof(argv[0]); // YAW
    target_v[1] = Shell::atof(argv[1]); // PITCH
    GimbalLG::set_action(GimbalLG::VELOCITY_MODE);
    GimbalLG::set_target_velocity(target_v[0],target_v[1]);
}

static void cmd_gimbal_set_target_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_angle yaw_angle pitch_angle");
        return;
    }
    GimbalLG::set_action(GimbalLG::ABS_ANGLE_MODE);
    GimbalLG::set_target_angle(Shell::atof(argv[0]),Shell::atof(argv[1]));
}

void cmd_gimbal_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp, "g_set_params yaw(0)/pitch(1) angle_to_v(0)/v_to_i(0) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    GimbalSKD::pid_params_t *p = nullptr;

    if(argv[0]-'0') {           //YAW
        if(argv[1]-'0') {       //a2v

        } else {                //v2i

        }
    } else {                    //PITCH
        if(argv[1]-'0') {       //a2v

        } else {                //v2i

        }
    }

    GimbalSKD::load_pid_params()

    GimbalSKD::pid_params_t yaw_a2v_params = ::a2v_pid[YAW].get_parameters();
    GimbalSKD::pid_params_t yaw_v2i_params = Gimbal::v2i_pid[YAW].get_parameters();
    GimbalSKD::pid_params_t pitch_a2v_params = Gimbal::a2v_pid[PITCH].get_parameters();
    GimbalSKD::pid_params_t pitch_v2i_params = Gimbal::v2i_pid[PITCH].get_parameters();

    Gimbal::pid_params_t *p = nullptr;
    if (*argv[0] == '0' && *argv[1] == '0') p = &yaw_a2v_params;
    else if (*argv[0] == '0' && *argv[1] == '1') p = &yaw_v2i_params;
    else if (*argv[0] == '1' && *argv[1] == '0') p = &pitch_a2v_params;
    else if (*argv[0] == '1' && *argv[1] == '1') p = &pitch_v2i_params;
    else {
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    *p = {Shell::atof(argv[2]),
          Shell::atof(argv[3]),
          Shell::atof(argv[4]),
          Shell::atof(argv[5]),
          Shell::atof(argv[6])};

    Gimbal::change_pid_params(yaw_a2v_params, yaw_v2i_params, pitch_a2v_params, pitch_v2i_params);

    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

ShellCommand gimbalControllerCommands[] = {
        {"g_enable",      cmd_gimbal_enable},
        {"g_enable_fb",   cmd_gimbal_enable_feedback},
        {"g_fix",         cmd_gimbal_fix_front_angle},
        {"g_set_v",       cmd_gimbal_set_target_velocities},
        {"g_set_angle",   cmd_gimbal_set_target_angle},
        {"g_set_params",  cmd_gimbal_set_parameters},
        {"g_echo_params", cmd_gimbal_echo_parameters},
        {"g_enable_fw",   cmd_gimbal_enable_fw},
        {nullptr,         nullptr}
};

int main() {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    // Enable power of bullet loader motor
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    // Enable power of ultraviolet lights
    palSetPadMode(GPIOH, GPIOH_POWER2_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER2_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1
    InspectorI::init(&can1, &can2, &ahrs);
    LED::all_off();

    /// Setup Shell
    Shell::start(THREAD_SHELL_PRIO);
    Shell::addCommands(mainProgramCommands);
    Shell::addCommands(gimbalControllerCommands);
    chThdSleepMilliseconds(50);  // wait for logo to print :)

    /// Setup SDCard
    if (SDCard::init()) {
        SDCard::read_all();
        LED::led_on(DEV_BOARD_LED_SD_CARD);  // LED 8 on if SD card inserted
    }

    LED::led_on(DEV_BOARD_LED_SYSTEM_INIT);  // LED 1 on now

    /// Setup CAN1 & CAN2
    can1.start(THREAD_CAN1_PRIO);
    can2.start(THREAD_CAN2_PRIO);
    chThdSleepMilliseconds(5);
    InspectorI::startup_check_can();  // check no persistent CAN Error. Block for 100 ms
    LED::led_on(DEV_BOARD_LED_CAN);  // LED 2 on now

    /// Setup On-Board AHRS
    Vector3D ahrs_bias;
    if (SDCard::get_data(MPU6500_BIAS_DATA_ID, &ahrs_bias, sizeof(ahrs_bias)) == SDCard::OK) {
        ahrs.load_calibration_data(ahrs_bias);
        LOG("Use AHRS bias in SD Card");
    } else {
        ahrs.load_calibration_data(MPU6500_STORED_GYRO_BIAS);
        LOG_WARN("Use default AHRS bias");
    }
    ahrs.start(ON_BOARD_AHRS_MATRIX_, THREAD_MPU_PRIO, THREAD_IST_PRIO, THREAD_AHRS_PRIO);
    chThdSleepMilliseconds(5);
    InspectorI::startup_check_mpu();  // check MPU6500 has signal. Block for 20 ms
    InspectorI::startup_check_ist();  // check IST8310 has signal. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_AHRS);  // LED 3 on now

    /// Setup Remote
    Remote::start();
    InspectorI::startup_check_remote();  // check Remote has signal. Block for 50 ms
    LED::led_on(DEV_BOARD_LED_REMOTE);  // LED 4 on now


    /// Setup GimbalIF (for Gimbal and Shoot)
    GimbalIF::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW,
                   GIMBAL_YAW_MOTOR_TYPE, GIMBAL_PITCH_MOTOR_TYPE, SHOOT_BULLET_MOTOR_TYPE);
    chThdSleepMilliseconds(2000);  // wait for C610 to be online and friction wheel to reset
    InspectorI::startup_check_gimbal_feedback(); // check gimbal motors has continuous feedback. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_GIMBAL);  // LED 5 on now


    /// Setup ChassisIF
    ChassisIF::init(&can1);
    chThdSleepMilliseconds(10);
    InspectorI::startup_check_chassis_feedback();  // check chassis motors has continuous feedback. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_CHASSIS);  // LED 6 on now


    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser

    /// Setup Referee
    Referee::init(THREAD_REFEREE_SENDING_PRIO);

    /// Setup VisionPort
    VisionPort::init();

    /// Setup SuperCapacitor Port
    SuperCapacitor::init(&can2);

    /// Complete Period 1
    LED::green_on();  // LED Green on now


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Echo Gimbal Raws and Converted Angles
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        GimbalIF::feedback[GimbalIF::YAW].last_angle_raw, GimbalIF::feedback[GimbalIF::YAW].actual_angle,
        GimbalIF::feedback[GimbalIF::PITCH].last_angle_raw, GimbalIF::feedback[GimbalIF::PITCH].actual_angle);

    /// Start SKDs
    GimbalSKD::start(&ahrs, GIMBAL_ANGLE_INSTALLATION_MATRIX_, GIMBAL_GYRO_INSTALLATION_MATRIX_,
                     GIMBAL_YAW_INSTALL_DIRECTION, GIMBAL_PITCH_INSTALL_DIRECTION, THREAD_GIMBAL_SKD_PRIO);
    GimbalSKD::load_pid_params(GIMBAL_PID_YAW_A2V_PARAMS, GIMBAL_PID_YAW_V2I_PARAMS,
                               GIMBAL_PID_PITCH_A2V_PARAMS, GIMBAL_PID_PITCH_V2I_PARAMS);
    GimbalSKD::set_yaw_restriction(GIMBAL_RESTRICT_YAW_MIN_ANGLE, GIMBAL_RESTRICT_YAW_MAX_ANGLE,
                                   GIMBAL_RESTRICT_YAW_VELOCITY);

    ShootSKD::start(SHOOT_BULLET_INSTALL_DIRECTION, ShootSKD::POSITIVE /* of no use */, THREAD_SHOOT_SKD_PRIO);
    ShootSKD::load_pid_params(SHOOT_PID_BULLET_LOADER_A2V_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS,
                              {0, 0, 0, 0, 0} /* of no use */, {0, 0, 0, 0, 0} /* of no use */);

    ChassisSKD::start(CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE, ChassisSKD::POSITIVE,
                      0, THREAD_CHASSIS_SKD_PRIO);
    ChassisSKD::load_pid_params(CHASSIS_FOLLOW_PID_THETA2V_PARAMS, CHASSIS_PID_V2I_PARAMS);

    /// Start LGs
    GimbalLG::init();
    ShootLG::init(SHOOT_DEGREE_PER_BULLET, THREAD_SHOOT_LG_STUCK_DETECT_PRIO, THREAD_SHOOT_BULLET_COUNTER_PRIO);
    ChassisLG::init(THREAD_CHASSIS_LG_DODGE_PRIO, CHASSIS_DODGE_MODE_THETA, CHASSIS_BIASED_ANGLE);


    /// Start Inspector and User Threads
    InspectorI::start_inspection(THREAD_INSPECTOR_PRIO);
    UserI::start(THREAD_USER_PRIO, THREAD_USER_ACTION_PRIO, THREAD_USER_CLIENT_DATA_SEND_PRIO, FEEDBACK_THREAD_PRIO);

    /// Complete Period 2
    Buzzer::play_sound(Buzzer::sound_startup_intel, THREAD_BUZZER_PRIO);  // Now play the startup sound


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