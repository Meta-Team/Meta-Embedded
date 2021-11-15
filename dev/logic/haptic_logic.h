//
// Created by 钱晨 on 11/4/21.
//

#ifndef META_INFANTRY_HAPTIC_LOGIC_H
#define META_INFANTRY_HAPTIC_LOGIC_H

#include "can_motor_scheduler.h"
#include "module/low_pass_filter.hpp"

using namespace chibios_rt;

class haptic_logic {
public:

    static void init(tprio_t PRIO, tprio_t FBPRIO);
    /**
     * @brief Max torque current threshold for motor, disables PID controllers, enters torque mode.
     * @details To provide a seamless user experience, the target torque current for torque mode should be the same
     *          as max torque current threshold.
     * */
    static int current_threshold;
    /**
     * @brief Min velocity threshold for motor, enables PID controllers, enters 2-Loop PID angle control mode.
     * */
    static float velocity_threshold;
    /**
     * @brief Target angle for 2-Loop PID angle control mode.
     * */
    static float target_angle;

    enum mode_t {
        torqueMode,
        velMode,
        angleMode,
        zeroVelMode
    };

    static mode_t HAPTIC_DVC_MODE;

    static LowPassFilteredValue LowPassFilter[CANBUS_MOTOR_CFG::MOTOR_COUNT];

    class back_driveability_thread : public BaseStaticThread<512> {
        void main() final;
    };
    static back_driveability_thread BackDriveabilityThd;

    class button_switch_thread : public BaseStaticThread<512> {
        void main() final;
    };
    static button_switch_thread ButtonSwitchThread;
};


#endif //META_INFANTRY_HAPTIC_LOGIC_H
