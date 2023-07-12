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
    // nullptr means using the default configuration, which is advised
    Rudder rudder1(&PWMD4,nullptr,0,Rudder::MG995);

    rudder1.start();
    rudder1.set_rudder_angle(90);
    while(1){
        chThdSleepMilliseconds(2000);
        rudder1.set_rudder_angle(0);
        chThdSleepMilliseconds(2000);
        rudder1.set_rudder_angle(90);
        chThdSleepMilliseconds(2000);
        rudder1.set_rudder_angle(180);
        chThdSleepMilliseconds(2000);
        rudder1.set_rudder_angle(90);

    }
}

