//
// Created by Kerui Zhu on 7/15/2019.
//

#ifndef META_INFANTRY_ENGINEER_LOGIC_H
#define META_INFANTRY_ENGINEER_LOGIC_H

#include "ch.hpp"
#include "hal.h"
#include "engineer_elevator_skd.h"
#include "engineer_chassis_skd.h"
#include "robotic_arm_skd.h"
#include "dms_interface.h"
#include "referee_interface.h"

class EngineerLogic {

public:

    class EngineerLogicThread: public chibios_rt::BaseStaticThread<512>{
        void main()final ;
    };

    static bool ignore_DMS;

    static EngineerLogicThread engineerLogicThread;

    static void elevate_up();

    static void elevate_down();

    static void set_vx_direction(int v_id, int direction);

private:

    enum engineer_state_t{
        LOCK,
        FREE,
        FORWARD_ONLY,
        BACKWARD_ONLY,
        RIIGHT_ONLY,
        LEFT_ONLY,
        BL_ROTATE,
        BR_ROTATE
    };

    static engineer_state_t state;

    static bool elevated;

    static uint8_t edges;

    static uint16_t trigger_height;

    static void update_dms_data();
};


#endif //META_INFANTRY_ENGINEER_LOGIC_H
