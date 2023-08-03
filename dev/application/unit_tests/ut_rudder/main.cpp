//
// Created by Wu Feiyang on 2023/7/11.
//

/**
 * @brief This is example code to use the Rudder Interface.
 */

#include "hal.h"
#include "ch.hpp"
#include "rudder_interface.h"


int main(){
    halInit();
    chibios_rt::System::init();

    Rudder::init();
    Rudder rudder1(&PWMD4,0,Rudder::MG995);
    Rudder rudder2(&PWMD4,1,Rudder::MG995);
    rudder1.enable();
    rudder2.enable();
    Rudder::start(&PWMD4, nullptr);
    rudder1.set_rudder_angle(0);
    rudder2.set_rudder_angle(90);
    while(1){
        chThdSleepMilliseconds(2000);
        rudder1.set_rudder_angle(60);
        rudder2.set_rudder_angle(0);
        chThdSleepMilliseconds(2000);
        rudder1.set_rudder_angle(120);
        rudder2.set_rudder_angle(90);
        chThdSleepMilliseconds(2000);
        rudder1.set_rudder_angle(60);
        rudder2.set_rudder_angle(180);
        chThdSleepMilliseconds(2000);
        rudder1.set_rudder_angle(0);
        rudder2.set_rudder_angle(90);


    }
}

