//
// Created by Kerui Zhu on 7/11/2019.
//

#ifndef META_INFANTRY_ROBOTIC_ARM_SKD_H
#define META_INFANTRY_ROBOTIC_ARM_SKD_H

#include "ch.hpp"
#include "hal.h"
#include "pid_controller.hpp"
#include "vehicle/engineer/vehicle_engineer.h"
#include "engineer_interface.h"
#include "air_tank_interface.h"
/**
 * Fetch process:
 * 1.   Initially, the robotic arm is stretching out
 * 2.   When Engineer reaches a box and finds a proper position, clamp is activated
 * 3.   When the clamp holds fast and the operator thinks it is ready,
 *          the robotic arm rotates a certain angle and collects the bullets,
 *          holds for about 3 seconds and rotates back to front,
 *          when reaching a trigger angle, release the box
 */

#define POWER_PAD GPIOH_POWER4_CTRL

class RoboticArmSKD{

public:

    class RoboticArmThread: public chibios_rt::BaseStaticThread<256>{
        void main()final;
    };

    static RoboticArmThread roboticArmThread;

    static int extend_state;
    static int clamp_state;
    static int slide_x_state;

    static int should_set_arm_normal;       //used in RoboticArmLG

    enum robotic_arm_state_t {
        NORMAL,
        COLLECT_BULLET
    };

    enum bullet_state_t{
        WAITING,
        BOX_CLAMPED,
        TAKING_BULLET
    };

    static bool released;

    //TODO: lock()

private:

public:

    /**
     * @brief perform action on clamp
     * @param target_status
     */

    static void start(tprio_t skd_thread_prio);

    static void next_step();
    static void prev_step();

    /**
     * a function used by engineer_auto_logic
     * @param clamp
     * @param slide_y
     * @param slide_x
     * if the input is -1, it means that the value should be maintained
     */
    static void set_air_tank(int clamp, int slide_y, int slide_x);

    static robotic_arm_state_t state;

    static bullet_state_t bullet_state;

    static void stretch_out();

    static void pull_back();

private:

    static float trigger_angle;

    static float target_velocity[2];

    static PIDController v2i_pid[2];

    static void update_target_current();


    friend void cmd_robotic_arm_set_v2i(BaseSequentialStream *chp, int argc, char *argv[]);

};

#endif //META_INFANTRY_ROBOTIC_ARM_SKD_H
