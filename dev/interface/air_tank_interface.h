//
// Created by zzb on 2020/1/18.
//

#ifndef META_INFANTRY_AIR_TANK_INTERFACE_H
#define META_INFANTRY_AIR_TANK_INTERFACE_H

#include "hal.h"

#define CLAMP_L GPIOE_PIN6         //Pin K1 - PE6
#define CLAMP_R GPIOE_PIN12        //Pin K2 - PE12
#define SLIDE_X_1_L GPIOE_PIN5     //Pin J1 - PE5
#define SLIDE_X_1_R GPIOE_PIN4     //Pin J2 - PE4
#define SLIDE_X_2_L GPIOF_PIN1     //Pin I1 - PF1
#define SLIDE_X_2_R GPIOF_PIN0     //Pin I2 - PF0
#define SLIDE_Y_L GPIOC_PIN2       //Pin L1 - PC2
#define SLIDE_Y_R GPIOB_PIN0       //Pin L2 - PB0

class air_tank_base{
public:
    /**
     * There are four air tanks in the engineer:
     * CLAMP: the one which controls the clamp;
     * SLIDE_X_1 and SLIDE_X_2: the two air tanks which control the x axis of the slide way
     * SLIDE_Y: the one which controls the y axis of the slide way
     */
    enum air_tank_id{
        CLAMP,
        SLIDE_X_1,
        SLIDE_X_2,
        SLIDE_Y,
        TANK_COUNT
    };
};
class air_tank_interface: public air_tank_base{
public:
    /**
     * val = 0: don't use the tank;
     * val = 1: use the tank;
     */
    static void init();
    static void set_tank(int id, int val);
    static int get_status(int id);

private:
    static int status[TANK_COUNT];
};


#endif //META_INFANTRY_AIR_TANK_INTERFACE_H
