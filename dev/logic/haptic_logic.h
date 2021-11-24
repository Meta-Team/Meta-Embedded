//
// Created by 钱晨 on 11/4/21.
//

#ifndef META_INFANTRY_HAPTIC_LOGIC_H
#define META_INFANTRY_HAPTIC_LOGIC_H

#include "can_motor_scheduler.h"
#include "module/low_pass_filter.hpp"

#define CALIB_TIMEOUT_DELAY 5000
#define CALIB_THRESHOLD_DELAY 500

using namespace chibios_rt;

class HapticLG {
public:

    static void init(tprio_t PRIO, tprio_t BTNPRIO);
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
        calibrateMode,
        followMode,
        zeroVelMode
    };

    static mode_t haptic_device_mode;

    static LowPassFilteredValue LowPassFilter[CANMotorCFG::MOTOR_COUNT];

    class BackDrivabilityThread : public BaseStaticThread<512> {
        bool calibrated = false;
        void main() final;
    };
    static BackDrivabilityThread back_drivability_thd;

    class ButtonSwitchThread : public BaseStaticThread<512> {
        void main() final;
    };
    static ButtonSwitchThread btn_switch_thd;
};


#endif //META_INFANTRY_HAPTIC_LOGIC_H
