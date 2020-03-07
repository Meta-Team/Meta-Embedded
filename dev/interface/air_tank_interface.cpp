//
// Created by zzb on 2020/1/18.
// Revised by Kerui Zhu on 2020/2/21
//

#include "air_tank_interface.h"

bool AirTankIF::status[TANK_COUNT];

void AirTankIF::init(){
    //set all the air tanks to be 0
    for (int i = 0; i <= SLIDE_Y; i++){
        status[i] = false;
    }
    palClearPad(CLAMP_L_GPIO, CLAMP_L);
    palClearPad(CLAMP_R_GPIO, CLAMP_R);
    palClearPad(SLIDE_X_1_L_GPIO, SLIDE_X_1_L);
    palClearPad(SLIDE_X_1_R_GPIO, SLIDE_X_1_R);
    palClearPad(SLIDE_X_2_L_GPIO, SLIDE_X_2_L);
    palClearPad(SLIDE_X_2_R_GPIO, SLIDE_X_2_R);
    palClearPad(SLIDE_Y_L_GPIO, SLIDE_Y_L);
    palClearPad(SLIDE_Y_R_GPIO, SLIDE_Y_R);

}

void AirTankIF::set_tank(air_tank_id id, bool set_on){
    if (id > SLIDE_Y || id < 0 || status[id] == set_on) return;

    //change status of the air tank
    status[id] = set_on;
    uint8_t pal_bit_left, pal_bit_right;

    if (status[id]){
        pal_bit_left = PAL_HIGH;
        pal_bit_right = PAL_LOW;
    } else{
        pal_bit_left = PAL_LOW;
        pal_bit_right = PAL_HIGH;
    }

    switch (id){
        case CLAMP:
            palWritePad(CLAMP_L_GPIO, CLAMP_L, pal_bit_left);
            palWritePad(CLAMP_R_GPIO, CLAMP_R, pal_bit_right);
            break;
        case SLIDE_X_1:
            palWritePad(SLIDE_X_1_L_GPIO, SLIDE_X_1_L, pal_bit_left);
            palWritePad(SLIDE_X_1_R_GPIO, SLIDE_X_1_R, pal_bit_right);
            break;
        case SLIDE_X_2:
            palWritePad(SLIDE_X_2_L_GPIO, SLIDE_X_2_L, pal_bit_left);
            palWritePad(SLIDE_X_2_R_GPIO, SLIDE_X_2_R, pal_bit_right);
            break;
        case SLIDE_Y:
            palWritePad(SLIDE_Y_L_GPIO, SLIDE_Y_L, pal_bit_left);
            palWritePad(SLIDE_Y_R_GPIO, SLIDE_Y_R, pal_bit_right);
            break;
        default:
            break;
    }
}

bool AirTankIF::get_status(air_tank_id id){
    return id >= 0 && id < TANK_COUNT && status[id];
}