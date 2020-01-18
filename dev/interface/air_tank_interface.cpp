//
// Created by zzb on 2020/1/18.
//

#include "air_tank_interface.h"
int air_tank_interface::status[TANK_COUNT];

void air_tank_interface::init(){
    //set all the air tanks to be 0
    for (int i = 0; i <= SLIDE_Y; i++){
        status[i] = 0;
    }
    palClearPad(GPIOE, CLAMP_L);
    palClearPad(GPIOE, CLAMP_R);
    palClearPad(GPIOE, SLIDE_X_1_L);
    palClearPad(GPIOE, SLIDE_X_1_R);
    palClearPad(GPIOF, SLIDE_X_2_L);
    palClearPad(GPIOF, SLIDE_X_2_R);
    palClearPad(GPIOC, SLIDE_Y_L);
    palClearPad(GPIOB, SLIDE_Y_R);

}

void air_tank_interface::set_tank(int id, int val){
    if (get_status(id) == val) return;
    if (id > SLIDE_Y || id < 0) return;
    else{
        //change status of the air tank
        status[id] = (status[id] == 0)? 1 : 0;
        if (id == CLAMP){
            if (val == 1){
                palWritePad(GPIOE,CLAMP_L,PAL_HIGH);
                palWritePad(GPIOE,CLAMP_R,PAL_LOW);
            }
            else{
                palWritePad(GPIOE,CLAMP_L,PAL_LOW);
                palWritePad(GPIOE,CLAMP_R,PAL_HIGH);
            }
        }
        else if (id == SLIDE_X_1){
            if (val == 1){
                palWritePad(GPIOE,SLIDE_X_1_L,PAL_HIGH);
                palWritePad(GPIOE,SLIDE_X_1_R,PAL_LOW);
            }
            else{
                palWritePad(GPIOE,SLIDE_X_1_L,PAL_LOW);
                palWritePad(GPIOE,SLIDE_X_1_R,PAL_HIGH);
            }
        }
        else if (id == SLIDE_X_2){
            if (val == 1){
                palWritePad(GPIOF,SLIDE_X_2_L,PAL_HIGH);
                palWritePad(GPIOF,SLIDE_X_2_R,PAL_LOW);
            }
            else{
                palWritePad(GPIOF,SLIDE_X_2_L,PAL_LOW);
                palWritePad(GPIOF,SLIDE_X_2_R,PAL_HIGH);
            }
        }
        else if (id == SLIDE_Y){
            if (val == 1){
                palWritePad(GPIOC,SLIDE_Y_L,PAL_HIGH);
                palWritePad(GPIOB,SLIDE_Y_R,PAL_LOW);
            }
            else{
                palWritePad(GPIOC,SLIDE_Y_L,PAL_LOW);
                palWritePad(GPIOB,SLIDE_Y_R,PAL_HIGH);
            }
        }
    }
}

int air_tank_interface::get_status(int id){
    return status[id];
}