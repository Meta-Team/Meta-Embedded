//
// Created by admin on 2019/4/29.
//

#include "sentry_chassis_calculator.h"

bool SentryChassisCalculator::enable = false;
bool SentryChassisCalculator::go_right = false;
PIDController SentryChassisCalculator::dist_to_v;
PIDController SentryChassisCalculator::v_to_i;


void SentryChassisCalculator::init_calculator(CANInterface* can_interface) {
    init(can_interface);
    reset_present_position();
}

void SentryChassisCalculator::reset_present_position() {
    present_position = 0;
}

void SentryChassisCalculator::set_direction(bool rightOrLeft) {
    go_right = rightOrLeft;
}
