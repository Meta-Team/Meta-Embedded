//
// Created by 钱晨 on 2019-07-03.
//

/// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer.h"
#include "common_macro.h"

#include "shell.h"
#include "can_interface.h"
#include "ahrs.h"
#include "remote_interpreter.h"

#include "gimbal_interface.h"
#include "gimbal_scheduler.h"
#include "shoot_scheduler.h"
#include "gimbal_logic.h"
#include "hero_shoot_logic.h"
#include "infantry_shoot_logic.h"

#include "chassis_interface.h"
#include "chassis_scheduler.h"
#include "chassis_logic.h"

// Check if it is only used for hero.
#if defined(HERO)
#include "vehicle_hero.h"
#else
#error "main_hero.cpp should be only used for HERO. Hia! Hia! Hia!"
#endif
// Board Guard.
#if defined(BOARD_RM_2018_A)
#else
#error "HERO supports only RM Board 2018 A currently. Let's expect the new Board!"
#endif

/// Instances
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
//AHRSOnBoard ahrs;

int main(){
    return 0;
}