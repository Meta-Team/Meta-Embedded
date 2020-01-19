//
// Created by 18767 on 2020/1/20.
//

#ifndef META_INFANTRY_ENGINEER_AUTO_LOGIC_H
#define META_INFANTRY_ENGINEER_AUTO_LOGIC_H

#include"robotic_arm_skd.h"
#include"engineer_rescue_skd.h"
#include"engineer_interface.h"
#include "ch.hpp"
#include "hal.h"

class engineer_auto_logic {
public :
    enum grab_mode_t{
        none,
        mode1,
        mode2
    };

    //granMode == 0: don't grab
    //1: grab_mode1
    //2: grab_mode2
    static int grabMode;

    static void set_mode(int mode_);

    static void grab_mode2();

    static void grab_mode1();

    static void stop_using_arm();

    friend class EngineerAutoThread;

private:
    class EngineerAutoThread:  public chibios_rt::BaseStaticThread<512>{
    private:
        void main() final;
    public:
        //
    };
    EngineerAutoThread engineerAutoThread;
};


#endif //META_INFANTRY_ENGINEER_AUTO_LOGIC_H
