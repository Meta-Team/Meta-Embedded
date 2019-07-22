//
// Created by Kerui Zhu on 7/11/2019.
//

#ifndef META_INFANTRY_ROBOTIC_ARM_SKD_H
#define META_INFANTRY_ROBOTIC_ARM_SKD_H

#include "ch.hpp"
#include "hal.h"
#include "robotic_arm_interface.h"
#include "pid_controller.hpp"
#include "vehicle/engineer/vehicle_engineer.h"

/**
 * Fetch process:
 * 1.   Initially, the robotic arm is stretching out
 * 2.   When Engineer reaches a box and finds a proper position, clamp is activated
 * 3.   When the clamp holds fast and the operator thinks it is ready,
 *          the robotic arm rotates a certain angle and collects the bullets,
 *          holds for about 3 seconds and rotates back to front,
 *          when reaching a trigger angle, release the box
 */

// TODO: embed as constants
#define DOOR_PAD GPIOE_PIN4
#define LIFT_PAD GPIOE_PIN5
#define EXTEND_PAD GPIOE_PIN6
#define CLAMP_PAD GPIOE_PIN12
#define POWER_PAD GPIOH_POWER4_CTRL

class RoboticArmSKD {

public:

    class RoboticArmThread: public chibios_rt::BaseStaticThread<256>{
        void main()final;
    };

    static RoboticArmThread roboticArmThread;

    enum digital_status_t {
        LOW_STATUS = PAL_LOW,
        HIGH_STATUS = PAL_HIGH
    };

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

    static void init();

public:

    /**
     * @brief perform action on clamp
     * @param target_status
     */
    static void next_step();
    static void prev_step();

    static void change_extend();
    static void change_door();

    static digital_status_t door_state, lift_state, extend_state, clamp_state;

    static void change_digital_status(digital_status_t& status, uint8_t pad);

    static void set_digital_status(digital_status_t& status, uint8_t pad, digital_status_t state);

private:

    static robotic_arm_state_t state;

    static bullet_state_t bullet_state;

    static float trigger_angle;

    static float target_velocity;

    static PIDController v2i_pid;


    static void update_target_current();

    static void stretch_out();

    static void pull_back();

    friend void cmd_robotic_arm_set_v2i(BaseSequentialStream *chp, int argc, char *argv[]);

};

#endif //META_INFANTRY_ROBOTIC_ARM_SKD_H
