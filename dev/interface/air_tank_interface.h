//
// Created by zzb on 2020/1/18.
// Revised by Kerui Zhu on 2020/2/21
//

#ifndef META_INFANTRY_AIR_TANK_INTERFACE_H
#define META_INFANTRY_AIR_TANK_INTERFACE_H

#include "hal.h"

#define CLAMP_L_GPIO        GPIOE
#define CLAMP_R_GPIO        GPIOE
#define SLIDE_X_1_L_GPIO    GPIOE
#define SLIDE_X_1_R_GPIO    GPIOE
#define SLIDE_X_2_L_GPIO    GPIOF
#define SLIDE_X_2_R_GPIO    GPIOF
#define SLIDE_Y_L_GPIO      GPIOC
#define SLIDE_Y_R_GPIO      GPIOB

#define CLAMP_L         GPIOE_PIN6      //Pin K1 - PE6
#define CLAMP_R         GPIOE_PIN12     //Pin K2 - PE12
#define SLIDE_X_1_L     GPIOE_PIN5      //Pin J1 - PE5
#define SLIDE_X_1_R     GPIOE_PIN4      //Pin J2 - PE4
#define SLIDE_X_2_L     GPIOF_PIN1      //Pin I1 - PF1
#define SLIDE_X_2_R     GPIOF_PIN0      //Pin I2 - PF0
#define SLIDE_Y_L       GPIOC_PIN2      //Pin L1 - PC2
#define SLIDE_Y_R       GPIOB_PIN0      //Pin L2 - PB0

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

class AirTankIF{
public:
    static void init();

     /**
      * set_tank
      * @brief Implement tank operation
      * @param id The tank to be operated
      * @param set_on If true, turn on the tank; otherwise, turn off
      */
    static void set_tank(air_tank_id id, bool set_on);

    /**
     * get_status
     * @brief get tank status
     * @param id The tank to be read status
     * @return Whether the tank is turned on or not
     */
    static bool get_status(air_tank_id id);

private:
    static bool status[TANK_COUNT];
};


#endif //META_INFANTRY_AIR_TANK_INTERFACE_H
