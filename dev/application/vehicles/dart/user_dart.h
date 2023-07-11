//
// Created by Wu Feiyang on 2023/7/11.
//
#ifndef META_EMBEDDED_USER_DART_H
#define META_EMBEDDED_USER_DART_H
#include "ch.hpp"
#include "can_interface.h"
#include "can_motor_interface.h"
#include "can_motor_feedback.h"
#include "can_motor_controller.h"
#include "remote_interpreter.h"
#include "referee_interface.h"


class UserDart{
public:
    static void start(tprio_t user_thd_prio);
private:
    class UserThread:public BaseStaticThread<512>{
    private:
        void main() final;
        CANMotorFeedback feedback;
    };
    static UserThread userThread;
};


#endif //META_EMBEDDED_USER_DART_H