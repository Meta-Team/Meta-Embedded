//
// Created by 18767 on 2020/1/20.
//

#ifndef META_INFANTRY_ROBOTIC_ARM_LOGIC_H
#define META_INFANTRY_ROBOTIC_ARM_LOGIC_H

#include"new_robotic_arm_skd.h"
#include"engineer_rescue_skd.h"
#include "ch.hpp"
#include "hal.h"

class RoboticArmLG {

public :

    enum grab_state_t{

    };

    class RoboticArmLGThread:  public chibios_rt::BaseStaticThread<256>{
        void main() final;
    };
    static RoboticArmLGThread roboticArmLgThread;

    static void init();

    static void get_prepared();

    static void start_grabing();

    static void finish();

    static void auto_logic_enable(bool enable);

    static void next_step();

    static void previous_step();

private :
    static bool auto_logic_enabled;
    static bool grabing;
};


#endif //META_INFANTRY_ROBOTIC_ARM_LOGIC_H
