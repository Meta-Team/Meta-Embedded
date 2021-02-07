//
// Created by ... on YYYY/MM/DD.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "remote_interpreter.h"

#include "debug/shell/shell.h"
#include "interface/motor_interface.h"
#include "module/pid_controller.hpp"

#include "scheduler/buzzer_scheduler.h"

enum motor_id_t {
    YAW = 0,
    PITCH = 1,
    BULLET = 2,
    FW_LEFT = 3,
    FW_RIGHT = 4,
    MOTOR_COUNT = 5
};

#define GIMBAL_YAW_FRONT_ANGLE_RAW 3457
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 7313  // of no use now

#define GIMBAL_YAW_CAN_CHANNEL      (MotorIF::can_channel_2)
#define GIMBAL_PITCH_CAN_CHANNEL    (MotorIF::can_channel_1)
#define GIMBAL_BULLET_CAN_CHANNEL   (MotorIF::can_channel_1)
#define GIMBAL_FW_LEFT_CAN_CHANNEL  (MotorIF::can_channel_1)
#define GIMBAL_FW_RIGHT_CAN_CHANNEL (MotorIF::can_channel_1)

#define GIMBAL_YAW_CAN_ID         4
#define GIMBAL_PITCH_CAN_ID       5
#define GIMBAL_BULLET_CAN_ID      6
#define GIMBAL_FW_LEFT_CAN_ID     2
#define GIMBAL_FW_RIGHT_CAN_ID    3

#define GIMBAL_YAW_MOTOR_TYPE      (CANInterface::GM6020)
#define GIMBAL_PITCH_MOTOR_TYPE    (CANInterface::GM6020)
#define SHOOT_BULLET_MOTOR_TYPE    (CANInterface::M2006)
#define SHOOT_PLATE_MOTOR_TYPE     (CANInterface::NONE_MOTOR)
#define GIMBAL_FW_LEFT_MOTOR_TYPE  (CANInterface::M3508)
#define GIMBAL_FW_RIGHT_MOTOR_TYPE (CANInterface::M3508)

#define GIMBAL_INTERFACE_MAX_CURRENT 5000

#define GIMBAL_MOTOR_CONFIG \
{ {GIMBAL_YAW_CAN_CHANNEL,          GIMBAL_YAW_CAN_ID,          GIMBAL_YAW_MOTOR_TYPE,         GIMBAL_INTERFACE_MAX_CURRENT,    1,   GIMBAL_YAW_FRONT_ANGLE_RAW},\
  {GIMBAL_PITCH_CAN_CHANNEL,        GIMBAL_PITCH_CAN_ID,        GIMBAL_PITCH_MOTOR_TYPE,       GIMBAL_INTERFACE_MAX_CURRENT,    1,   GIMBAL_PITCH_FRONT_ANGLE_RAW},\
  {GIMBAL_BULLET_CAN_CHANNEL,       GIMBAL_BULLET_CAN_ID,       SHOOT_BULLET_MOTOR_TYPE,       GIMBAL_INTERFACE_MAX_CURRENT,    0,  0},\
  {GIMBAL_FW_LEFT_CAN_CHANNEL,      GIMBAL_FW_LEFT_CAN_ID,      GIMBAL_FW_LEFT_MOTOR_TYPE,     GIMBAL_INTERFACE_MAX_CURRENT,    0,  0},\
  {GIMBAL_FW_RIGHT_CAN_CHANNEL,     GIMBAL_FW_RIGHT_CAN_ID,     GIMBAL_FW_RIGHT_MOTOR_TYPE,    GIMBAL_INTERFACE_MAX_CURRENT,    0,  0} }                         \

/// Thread Priority List
#define THREAD_CAN1_FEEDBACK_PRIO           (HIGHPRIO - 1)
#define THREAD_CAN1_CURRENT_PRIO            (HIGHPRIO - 2)
#define THREAD_CAN2_FEEDBACK_PRIO           (HIGHPRIO - 3)
#define THREAD_CAN2_CURRENT_PRIO            (HIGHPRIO - 4)
#define THREAD_GIMBAL_SKD_PRIO              (NORMALPRIO + 3)
#define THREAD_FEEDBACK_PRIO                (LOWPRIO + 6)
#define THREAD_SHELL_PRIO                   (LOWPRIO + 5)
#define THREAD_BUZZER_SKD_PRIO              (LOWPRIO)


/// Gimbal and Shoot PID Parameters
#define GIMBAL_PID_YAW_A2V_KP 12.0f
#define GIMBAL_PID_YAW_A2V_KI 0.0f
#define GIMBAL_PID_YAW_A2V_KD 0.08f
#define GIMBAL_PID_YAW_A2V_I_LIMIT 1000.0f
#define GIMBAL_PID_YAW_A2V_OUT_LIMIT 2000.0f
#define GIMBAL_PID_YAW_A2V_PARAMS \
    {GIMBAL_PID_YAW_A2V_KP, GIMBAL_PID_YAW_A2V_KI, GIMBAL_PID_YAW_A2V_KD, \
    GIMBAL_PID_YAW_A2V_I_LIMIT, GIMBAL_PID_YAW_A2V_OUT_LIMIT}

#define GIMBAL_PID_YAW_V2I_KP 36.0f
#define GIMBAL_PID_YAW_V2I_KI 0.9f
#define GIMBAL_PID_YAW_V2I_KD 0.0f
#define GIMBAL_PID_YAW_V2I_I_LIMIT 2500.0f
#define GIMBAL_PID_YAW_V2I_OUT_LIMIT 10000.0f
#define GIMBAL_PID_YAW_V2I_PARAMS \
    {GIMBAL_PID_YAW_V2I_KP, GIMBAL_PID_YAW_V2I_KI, GIMBAL_PID_YAW_V2I_KD, \
    GIMBAL_PID_YAW_V2I_I_LIMIT, GIMBAL_PID_YAW_V2I_OUT_LIMIT}

#define GIMBAL_PID_PITCH_A2V_KP 10.0f
#define GIMBAL_PID_PITCH_A2V_KI 0.0f
#define GIMBAL_PID_PITCH_A2V_KD 0.2f
#define GIMBAL_PID_PITCH_A2V_I_LIMIT 60.0f
#define GIMBAL_PID_PITCH_A2V_OUT_LIMIT 90.0f
#define GIMBAL_PID_PITCH_A2V_PARAMS \
    {GIMBAL_PID_PITCH_A2V_KP, GIMBAL_PID_PITCH_A2V_KI, GIMBAL_PID_PITCH_A2V_KD, \
    GIMBAL_PID_PITCH_A2V_I_LIMIT, GIMBAL_PID_PITCH_A2V_OUT_LIMIT}

#define GIMBAL_PID_PITCH_V2I_KP 75.0f
#define GIMBAL_PID_PITCH_V2I_KI 0.85f
#define GIMBAL_PID_PITCH_V2I_KD 0.00f
#define GIMBAL_PID_PITCH_V2I_I_LIMIT 10000.0f
#define GIMBAL_PID_PITCH_V2I_OUT_LIMIT 20000.0f
#define GIMBAL_PID_PITCH_V2I_PARAMS \
    {GIMBAL_PID_PITCH_V2I_KP, GIMBAL_PID_PITCH_V2I_KI, GIMBAL_PID_PITCH_V2I_KD, \
    GIMBAL_PID_PITCH_V2I_I_LIMIT, GIMBAL_PID_PITCH_V2I_OUT_LIMIT}

#define SHOOT_PID_BULLET_LOADER_A2V_KP 10.0f  // a number large enough, see shoot speed note at ShootSKD
#define SHOOT_PID_BULLET_LOADER_A2V_KI 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_KD 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_OUT_LIMIT 360.0f  // will be replaced, see shoot speed note at ShootSKD
#define SHOOT_PID_BULLET_LOADER_A2V_PARAMS \
    {SHOOT_PID_BULLET_LOADER_A2V_KP, SHOOT_PID_BULLET_LOADER_A2V_KI, SHOOT_PID_BULLET_LOADER_A2V_KD, \
    SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT, SHOOT_PID_BULLET_LOADER_A2V_OUT_LIMIT}

#define SHOOT_PID_BULLET_LOADER_V2I_KP 35.0f
#define SHOOT_PID_BULLET_LOADER_V2I_KI 10.0f
#define SHOOT_PID_BULLET_LOADER_V2I_KD 0.0f
#define SHOOT_PID_BULLET_LOADER_V2I_I_LIMIT 8000.0f
#define SHOOT_PID_BULLET_LOADER_V2I_OUT_LIMIT 8000.0f
#define SHOOT_PID_BULLET_LOADER_V2I_PARAMS \
    {SHOOT_PID_BULLET_LOADER_V2I_KP, SHOOT_PID_BULLET_LOADER_V2I_KI, SHOOT_PID_BULLET_LOADER_V2I_KD, \
    SHOOT_PID_BULLET_LOADER_V2I_I_LIMIT, SHOOT_PID_BULLET_LOADER_V2I_OUT_LIMIT}

//TODO: Need to revised to the newest pid params.
#define SHOOT_PID_FW_LEFT_V2I_KP 26.0f
#define SHOOT_PID_FW_LEFT_V2I_KI 0.1f
#define SHOOT_PID_FW_LEFT_V2I_KD 0.02f
#define SHOOT_PID_FW_LEFT_V2I_I_LIMIT 2000.0f
#define SHOOT_PID_FW_LEFT_V2I_OUT_LIMIT 6000.0f
#define SHOOT_PID_FW_LEFT_V2I_PARAMS \
    {SHOOT_PID_FW_LEFT_V2I_KP, SHOOT_PID_FW_LEFT_V2I_KI, SHOOT_PID_FW_LEFT_V2I_KD, \
    SHOOT_PID_FW_LEFT_V2I_I_LIMIT, SHOOT_PID_FW_LEFT_V2I_OUT_LIMIT}

//TODO: Need to revised to the newest pid params.
#define SHOOT_PID_FW_RIGHT_V2I_KP 26.0f
#define SHOOT_PID_FW_RIGHT_V2I_KI 0.1f
#define SHOOT_PID_FW_RIGHT_V2I_KD 0.02f
#define SHOOT_PID_FW_RIGHT_V2I_I_LIMIT 2000.0f
#define SHOOT_PID_FW_RIGHT_V2I_OUT_LIMIT 6000.0f
#define SHOOT_PID_FW_RIGHT_V2I_PARAMS \
    {SHOOT_PID_FW_RIGHT_V2I_KP, SHOOT_PID_FW_RIGHT_V2I_KI, SHOOT_PID_FW_RIGHT_V2I_KD, \
    SHOOT_PID_FW_RIGHT_V2I_I_LIMIT, SHOOT_PID_FW_RIGHT_V2I_OUT_LIMIT}

using namespace chibios_rt;

CANInterface::motor_feedback_t *feedback[4];

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

bool remote_enable;
bool feedback_enable[MOTOR_COUNT];
const char *motor_name[] = {"yaw", "pitch", "bullet", "fw_left", "fw_right"};
float targetAngle[MOTOR_COUNT];
float targetVelocity[MOTOR_COUNT];
int targetCurrent[MOTOR_COUNT];

bool a2v_enable[MOTOR_COUNT];
bool v2i_enable[MOTOR_COUNT];
PIDController v2iController[MOTOR_COUNT];
PIDController a2vController[MOTOR_COUNT];

unsigned const GIMBAL_THREAD_INTERVAL = 1;    // [ms]
unsigned const GIMBAL_FEEDBACK_INTERVAL = 25; // [ms]

static MotorIF::motor_can_config_t GIMBAL_MOTOR_CONFIG_[MOTOR_COUNT] = GIMBAL_MOTOR_CONFIG;

static void cmd_enable_remote(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "remote_enable set_enable(0/1)");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    remote_enable = *argv[0] != '0';
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

static void cmd_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "fb_enable motor(yaw(0)/pitch(1)/bl(2)/fw_left(3)/fw_right(4)) set_enable(0/1)");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if (motor_id < 0 || motor_id > 4){
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    feedback_enable[motor_id] = *argv[1] != '0';
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

static void cmd_set_target_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "set_a motor(yaw(0)/pitch(1)/bl(2)) target_angle");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if (motor_id < 0 || motor_id > 2){
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    targetAngle[motor_id] = Shell::atof(argv[1]);
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

static void cmd_set_target_velocity(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "set_v motor(yaw(0)/pitch(1)/bl(2)/fw_left(3)/fw_right(4)) target_velocity");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if (motor_id < 0 || motor_id > 4){
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    targetVelocity[motor_id] = Shell::atof(argv[1]);
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

static void cmd_set_target_current(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "set_i motor(yaw(0)/pitch(1)/bl(2)/fw_left(3)/fw_right(4)) target_current");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if (motor_id < 0 || motor_id > 4){
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    targetCurrent[motor_id] = Shell::atoi(argv[1]);
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

static void cmd_enable_a2v(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "a2v_enable motor(yaw(0)/pitch(1)/bl(2)) set_enable(0/1)");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if (motor_id < 0 || motor_id > 2){
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    a2v_enable[motor_id] = *argv[1] != '0';
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

static void cmd_enable_v2i(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "v2i_enable motor(yaw(0)/pitch(1)/bl(2))/fw_left(3)/fw_right(4) set_enable(0/1)");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    int motor_id = Shell::atoi(argv[0]);
    if (motor_id < 0 || motor_id > 4){
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    v2i_enable[motor_id] = *argv[1] != '0';
    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

static void cmd_echo_status(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_status");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    chprintf(chp, "angle: %d %d %d" SHELL_NEWLINE_STR, MotorIF::feedback[YAW]->last_angle_raw, MotorIF::feedback[PITCH]->last_angle_raw, MotorIF::feedback[BULLET]->last_angle_raw);
    chprintf(chp, "target_angle: %d %d %d" SHELL_NEWLINE_STR, targetAngle[YAW], targetAngle[PITCH], targetAngle[BULLET]);
    chprintf(chp, "velocity: %.2f %.2f %.2f %.2f %.2f" SHELL_NEWLINE_STR, MotorIF::feedback[YAW]->actual_velocity, MotorIF::feedback[PITCH]->actual_velocity, MotorIF::feedback[BULLET]->actual_velocity, MotorIF::feedback[FW_LEFT]->actual_velocity, MotorIF::feedback[FW_RIGHT]->actual_velocity);
    chprintf(chp, "target_velocity: %.2f %.2f %.2f %.2f %.2f" SHELL_NEWLINE_STR, targetVelocity[YAW], targetVelocity[PITCH], targetVelocity[BULLET], targetVelocity[FW_LEFT], targetVelocity[FW_RIGHT]);
    chprintf(chp, "current: %d %d %d %d %d" SHELL_NEWLINE_STR, MotorIF::feedback[YAW]->actual_current, MotorIF::feedback[PITCH]->actual_current, MotorIF::feedback[BULLET]->actual_current, MotorIF::feedback[FW_LEFT]->actual_current, MotorIF::feedback[FW_RIGHT]->actual_current);
    chprintf(chp, "target_current: %d %d %d %d %d" SHELL_NEWLINE_STR, targetCurrent[YAW], targetCurrent[PITCH], targetCurrent[BULLET], targetCurrent[FW_LEFT], targetCurrent[FW_RIGHT]);
    chprintf(chp, "v2i_enable: %d %d %d %d %d" SHELL_NEWLINE_STR, v2i_enable[YAW], v2i_enable[PITCH], v2i_enable[BULLET], v2i_enable[FW_LEFT], v2i_enable[FW_RIGHT]);
    chprintf(chp, "a2v_enable: %d %d %d %d %d" SHELL_NEWLINE_STR, a2v_enable[YAW], a2v_enable[PITCH], a2v_enable[BULLET], a2v_enable[FW_LEFT], a2v_enable[FW_RIGHT]);
    chprintf(chp, "remote_enable: %d" SHELL_NEWLINE_STR, remote_enable);
}

static void cmd_set_param(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp, "set_pid motor(yaw(0)/pitch(1)/bl(2)/fw_left(3)/fw_right(4)) pid_id(angle_to_v(0)/v_to_i(1)) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    PIDController *p;
    p = (*argv[1] == '0') ? a2vController : v2iController;
    int motor_id = Shell::atoi(argv[0]);
    if (motor_id < 0 || (p == v2iController && motor_id > 4) || (p == a2vController && motor_id > 2)){
        shellUsage(chp, "param error" SHELL_NEWLINE_STR);
        return;
    }
    p[motor_id].change_parameters({Shell::atof(argv[2]),
                                   Shell::atof(argv[3]),
                                   Shell::atof(argv[4]),
                                   Shell::atof(argv[5]),
                                   Shell::atof(argv[6])});

    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

// Shell commands to ...
ShellCommand ShellCommands[] = {
        {"remote_enable", cmd_enable_remote},
        {"fb_enable", cmd_enable_feedback},
        {"set_a", cmd_set_target_angle},
        {"set_v", cmd_set_target_velocity},
        {"set_i", cmd_set_target_current},
        {"a2v_enable", cmd_enable_a2v},
        {"v2i_enable", cmd_enable_v2i},
        {"set_pid", cmd_set_param},
        {"echo_status", cmd_echo_status},
        {nullptr,    nullptr}
};


// Thread to ...
class SKDThread : public BaseStaticThread <512> {
private:
    void main() final {

        setName("SKDThread");
        while (!shouldTerminate()) {
            if (remote_enable) {
                if (!ABS_IN_RANGE(Remote::rc.wheel, 0.2)) {
                    targetVelocity[BULLET] = Remote::rc.wheel * 100.0f;
                    targetVelocity[FW_LEFT] = 2800.0f;
                    targetVelocity[FW_RIGHT] = -2800.0f;
                } else {
                    targetVelocity[BULLET] = targetVelocity[FW_LEFT] = targetVelocity[FW_RIGHT] = 0.0f;
                }
            }

            for (int i = 0; i < MOTOR_COUNT; i++){
                if (a2v_enable[i]){
                    targetVelocity[i] = a2vController[i].calc(MotorIF::feedback[i]->actual_angle, targetAngle[i]);
                }
                if (v2i_enable[i]){
                    targetCurrent[i] = (int)v2iController[i].calc(MotorIF::feedback[i]->actual_velocity, targetVelocity[i]);
                }
                MotorIF::set_target_current(targetCurrent[i], i);
            }
            sleep(TIME_MS2I(5));
        }
    }
} skdThread;

class FeedbackThread : public BaseStaticThread<512> {
private:
    void main() final {
        setName("feedback");
        while (!shouldTerminate()) {
            float actual_angle, target_angle, actual_velocity, target_velocity;
            for (int i = 0; i < MOTOR_COUNT; i++){
                if (feedback_enable[i]){
                    actual_angle = (a2v_enable[i]) ? MotorIF::feedback[i]->actual_angle : 0;
                    target_angle = (a2v_enable[i]) ? targetAngle[i] : 0;
                    actual_velocity = (v2i_enable[i]) ? MotorIF::feedback[i]->actual_velocity : 0;
                    target_velocity = (v2i_enable[i]) ? targetVelocity[i] : 0;
                    Shell::printf("fb %s %.2f %.2f %.2f %.2f %d %d" SHELL_NEWLINE_STR,
                                  motor_name[i],
                                  actual_angle, target_angle,
                                  actual_velocity, target_velocity,
                                  MotorIF::feedback[i]->actual_current, targetCurrent[i]);
                }
            }
            sleep(TIME_MS2I(GIMBAL_FEEDBACK_INTERVAL));
        }
    }
} feedbackThread;

int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(THREAD_SHELL_PRIO);
    Shell::addCommands(ShellCommands);

    can1.start(THREAD_CAN1_FEEDBACK_PRIO, THREAD_CAN1_CURRENT_PRIO);
    can2.start(THREAD_CAN2_FEEDBACK_PRIO, THREAD_CAN2_CURRENT_PRIO);

    Remote::start();

    MotorIF::init(&can1, &can2, GIMBAL_MOTOR_CONFIG_, MOTOR_COUNT);
    chThdSleepMilliseconds(2000);  // wait for C610 to be online and friction wheel to reset

    skdThread.start(THREAD_GIMBAL_SKD_PRIO);
    feedbackThread.start(THREAD_FEEDBACK_PRIO);

    v2iController[YAW].change_parameters(GIMBAL_PID_YAW_V2I_PARAMS);
    v2iController[PITCH].change_parameters(GIMBAL_PID_PITCH_V2I_PARAMS);
    v2iController[BULLET].change_parameters(SHOOT_PID_BULLET_LOADER_V2I_PARAMS);
    v2iController[FW_LEFT].change_parameters(SHOOT_PID_FW_LEFT_V2I_PARAMS);
    v2iController[FW_RIGHT].change_parameters(SHOOT_PID_FW_RIGHT_V2I_PARAMS);

    a2vController[YAW].change_parameters(GIMBAL_PID_YAW_A2V_PARAMS);
    a2vController[PITCH].change_parameters(GIMBAL_PID_PITCH_A2V_PARAMS);
    a2vController[BULLET].change_parameters(SHOOT_PID_BULLET_LOADER_A2V_PARAMS);

//    BuzzerSKD::init(THREAD_BUZZER_SKD_PRIO);
//    BuzzerSKD::play_sound(BuzzerSKD::sound_nyan_cat);

#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}

