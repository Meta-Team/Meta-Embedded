//
// Created by 404 on 2019-05-18.
//

// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"

#include "can_interface.h"
#include "common_macro.h"

#include "buzzer.h"
#include "mpu6500.h"
#include "remote_interpreter.h"

#include "gimbal.h"
#include "shoot.h"
#include "chassis.h"

#include "state_handler.h"

//Vehicle specific config
#if defined(HERO)

#include "vehicle_hero.h"

#else

#error "main_hero.cpp should only be used for Hero."

#endif

// Board guard
#if defined(BOARD_RM_2018_A)
#else
#error "HERO is only developed for RM board 2018 A."
#endif

#include "thread_gimbal.hpp"
#include "thread_shoot.hpp"
#include "thread_chassis.hpp"
#include "thread_error_detect.hpp"

// Interfaces
CANInterface can1(&CAND1);

//Threads
GimbalThread gimbalThread;
ShootThread shootThread;
ChassisThread chassisThread;
ErrorDetectThread errorDetectThread;

int main(void){

}