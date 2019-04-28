//
// Created by admin on 2019/4/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
#define META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H

#include "sentry_chassis_interface.h"
#include "pid_controller.h"
#include "can_interface.h"

class SentryChassisCalculator: public SentryChassis{
public:
    static bool enable;

    /**
     * @brief initialize the calculator class
     * @param can_interface
     */
    static void init_calculator(CANInterface* can_interface);

    /**
     * @brief set the present position to be the 0 point
     */
    static void reset_present_position();

    /**
     * @brief set the direction of the sentry movement
     * @param rightOrLeft true for right and false for left
     */
    void set_direction(bool rightOrLeft);
private:
    static bool go_right;
    static double present_position;
    static PIDController dist_to_v;
    static PIDController v_to_i;
};


#endif //META_INFANTRY_SENTRY_CHASSIS_CALCULATOR_H
