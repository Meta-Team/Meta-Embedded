//
// Created by Kerui Zhu on 2/17/2020.
//

#ifndef META_INFANTRY_NEW_ROBOTIC_ARM_SKD_H
#define META_INFANTRY_NEW_ROBOTIC_ARM_SKD_H

#include "ch.hpp"
#include "hal.h"
#include "pid_controller.hpp"
#include "vehicle/engineer/vehicle_engineer.h"
#include "engineer_interface.h"
#include "robotic_arm_interface.h"
#include "air_tank_interface.h"

#define POWER_PAD GPIOH_POWER4_CTRL


class NewRoboticArmSkd {

public:

    enum RA_motor_state_t{
        RETRIEVED,
        RETRIEVING,
        STRETCHING,
        STRETCHED
    };

    enum RA_motor_instruction{
        STRETCH,
        RETRIEVE
    };

    class RoboticArmThread: public chibios_rt::BaseStaticThread<256>{
        void main()final;
    };

    static RoboticArmThread roboticArmThread;

    static void start(tprio_t skd_thread_prio);

    static RA_motor_state_t get_motor_state();
    static void set_motor_instruction(RA_motor_instruction command);
    static bool is_extended();
    static void set_extension(bool extend);
    static bool is_clamped();
    static void set_clamp(bool clamp);
    static int get_x_slide_state();
    static void set_x_slide_state(int state);

private:
    static RA_motor_state_t motorState;
    static RA_motor_instruction targetState;
    static int x_slide_state;
    static float target_velocity;
    static float trigger_angle;
    static PIDController v2i_pid[2];
    static void update();
    static void calculate_current();
    friend void cmd_robotic_arm_set_v2i(BaseSequentialStream *chp, int argc, char *argv[]);

};


#endif //META_INFANTRY_NEW_ROBOTIC_ARM_SKD_H
