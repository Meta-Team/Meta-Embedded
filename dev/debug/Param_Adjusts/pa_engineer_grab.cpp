//
// Created by Qian Chen on 3/27/21.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "engineer_grab_skd.h"
#include "engineer_grab_mech_interface.h"
#include "remote_interpreter.h"
#include "vehicle/infantry/vehicle_infantry.h"
// Other headers here

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

PIDController::pid_params_t PIDParameter[2] = {SHOOT_PID_BULLET_LOADER_V2I_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS};

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_set_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 6) {
        shellUsage(chp, "set_pid MOTOR(LEFT/RIGHT) kp ki kd i_limit out_limit");
        return;
    }
    PIDController::pid_params_t NEW_Parameter = {Shell::atof(argv[1]), Shell::atof(argv[2]),
                                                 Shell::atof(argv[3]),Shell::atof(argv[4]),
                                                 Shell::atof(argv[5])};
    if(*argv[0] == 'L') {
        PIDParameter[0] = NEW_Parameter;
        engineerGrabSKD::load_v2i_pid_params(PIDParameter);
    } else if (*argv[0] == 'R') {
        PIDParameter[1] = NEW_Parameter;
        engineerGrabSKD::load_v2i_pid_params(PIDParameter);
    } else {
        shellUsage(chp, "set_pid MOTOR(LEFT/RIGHT) kp ki kd i_limit out_limit");
        return;
    }
    chprintf(chp, "pid_set!" SHELL_NEWLINE_STR);
}

static void cmd_echo_fb(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_fb");
        return;
    }
    Shell::printf("GrabberL :    %f" SHELL_NEWLINE_STR, EngGrabMechIF::feedback[EngGrabMechIF::GRABER_L]->accumulated_angle());
    Shell::printf("GrabberR :    %f" SHELL_NEWLINE_STR, EngGrabMechIF::feedback[EngGrabMechIF::GRABER_R]->accumulated_angle());
    Shell::printf("HAND     :    %f" SHELL_NEWLINE_STR, EngGrabMechIF::feedback[EngGrabMechIF::ROTATION_HAND]->accumulated_angle());
    Shell::printf("STATUS   :    %d" SHELL_NEWLINE_STR, engineerGrabSKD::echo_status());
}

static void cmd_rise(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "rise");
        return;
    }
    engineerGrabSKD::invoke_rising();
}

static void cmd_lower(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "lower");
        return;
    }
    engineerGrabSKD::invoke_lowering();
}

// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"set_pid", cmd_set_pid},
        {"echo_fb", cmd_echo_fb},
        {"rise",    cmd_rise},
        {"lower",   cmd_lower},
        {nullptr,    nullptr}
};


// Thread to ...
class ControlThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("pa_enginner_grab");
        while (!shouldTerminate()) {
            engineerGrabSKD::set_belt_target_velocity(Remote::rc.ch1 * 1000.0f);
            sleep(TIME_MS2I(100));
        }
    }
} controlThread;


int main(void) {

    halInit();
    System::init();
    Remote::start();
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    can1.start(NORMALPRIO+2,NORMALPRIO+3);
    can2.start(NORMALPRIO+4, NORMALPRIO+5);
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);
    EngGrabMechIF::motor_can_config_t CANCONFIG[5] = {  {EngGrabMechIF::can_channel_1, 0, CANInterface::M2006},
                                                        {EngGrabMechIF::can_channel_1, 1, CANInterface::M2006},
                                                        {EngGrabMechIF::can_channel_1, 2, CANInterface::M3508},
                                                        {EngGrabMechIF::can_channel_1, 3, CANInterface::M2006},
                                                        {EngGrabMechIF::can_channel_1, 4, CANInterface::M2006}};
    EngGrabMechIF::init(&can1, &can2, CANCONFIG);
    controlThread.start(NORMALPRIO + 1);
    engineerGrabSKD::install_direction direct[5] = {engineerGrabSKD::install_direction::NEGATIVE,
                                                    engineerGrabSKD::install_direction::POSITIVE,
                                                    engineerGrabSKD::install_direction::POSITIVE,
                                                    engineerGrabSKD::install_direction::NEGATIVE,
                                                    engineerGrabSKD::install_direction::POSITIVE};
    PIDController::pid_params_t pidParams[4] = {SHOOT_PID_BULLET_LOADER_V2I_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS,
                                                SHOOT_PID_BULLET_LOADER_V2I_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS};
    PIDController::pid_params_t pidParams1[2] = {SHOOT_PID_BULLET_LOADER_A2V_PARAMS, SHOOT_PID_BULLET_LOADER_A2V_PARAMS};
    engineerGrabSKD::start(NORMALPRIO+1, direct);
    engineerGrabSKD::load_v2i_pid_params(pidParams);
    engineerGrabSKD::load_a2v_pid_params(pidParams1);


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
