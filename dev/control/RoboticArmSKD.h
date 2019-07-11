//
// Created by Kerui Zhu on 7/11/2019.
//

#ifndef META_INFANTRY_ROBOTICARMSKD_H
#define META_INFANTRY_ROBOTICARMSKD_H

#include "ch.hpp"
#include "hal.h"
#include "robotic_arm_interface.h"
#include "pid_controller.hpp"
#include "vehicle/engineer/vehicle_engineer.h"

class RoboticArmSKD {

public:

    class RoboticArmThread: public chibios_rt::BaseStaticThread<256>{
        void main()final;
    };

    static RoboticArmThread roboticArmThread;

    enum clamp_status_t {
        CLAMP_RELAX = PAL_LOW,
        CLAMP_CLAMPED = PAL_HIGH
    };

    static clamp_status_t _clamp_status; // local storage

private:

    static void init();

public:

    /**
     * @brief perform action on clamp
     * @param target_status
     */
    static void set_clamp_action(clamp_status_t target_status);

    static void stretch_out();

    static void pull_back();

    static void update_target_current();

private:

    static bool released;

    static float trigger_angle;

    static float target_velocity;

    static PIDController v2i_pid;

};


#endif //META_INFANTRY_ROBOTICARMSKD_H
