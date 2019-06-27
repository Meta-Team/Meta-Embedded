//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_INFANTRY_USER_H
#define META_INFANTRY_USER_H

#include "ch.hpp"

#include "remote_interpreter.h"

#include "gimbal_logic.h"
#include "shoot_logic.h"
#include "chassis_logic.h"

#include "inspector.h"

class User {

public:

    static void start(tprio_t prio);

private:




    /// Chassis Parameters
    static constexpr float CHASSIS_COMMON_VX = 1200.0f;  // [mm/s]
    static constexpr float CHASSIS_COMMON_VY = 1200.0f;  // [mm/s]
    static constexpr float CHASSIS_COMMON_W = 360.0f;    // [degree/s]

    static constexpr float CHASSIS_PC_CTRL_RATIO = 0.5f;  // 50% when Ctrl is pressed


    /// Thread
    static constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]

    class UserThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static UserThread userThread;

};


#endif //META_INFANTRY_USER_H
