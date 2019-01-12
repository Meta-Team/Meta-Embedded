//
// Created by admin on 2019/1/12.
//

#include "shoot_interface.h"

/**
  * @brief set the friction wheel speed to 0 and set the bullet loader to unable
  */
void ShootInterface::set_zero_current() {
    loadEnable = false;  // Set the bullet loading unable
    fw_motor_duty_cycle = 0.0f;  // Set the friction wheel motor duty cycle to 0
    bl_current = 0;  // Set the bullet loader motor current to 0
}

/**
 * @brief use the ratio between the speed needed and the maximum speed to calculate the speed needed
 * @param speed_ratio       the ratio of the speed needed compared with the maximum speed
 */
void ShootInterface::set_shoot_mode(float speed_ratio) {
    if(speed_ratio > 0){

        // If the speed ratio is greater than 0

        fw_motor_duty_cycle = FW_FULL_SPEED_DUTY_CYCLE * speed_ratio;  // Set the friction wheel duty cycle
        loadEnable = (fw_motor_duty_cycle > BL_MOTOR_SPEED_DUTY_CYCLE_TRIGGER);  // Judge whether the speed of the friction wheel reaches the trigger speed
    } else{

        // If the speed ratio is not positive

        set_zero_current();  // Set the speed to 0
    }
}
/**
  * @brief send the shoot current to the corresponding friction wheel motors
  */
void ShootInterface::send_shoot_currents() {
    pwmEnableChannel(&FRICTION_WHEEL_PWM_TIM, FW_LEFT, PWM_PERCENTAGE_TO_WIDTH(&FRICTION_WHEEL_PWM_TIM, fw_motor_duty_cycle * 500 + 500));  // Send to the left friction wheel
    pwmEnableChannel(&FRICTION_WHEEL_PWM_TIM, FW_RIGHT, PWM_PERCENTAGE_TO_WIDTH(&FRICTION_WHEEL_PWM_TIM, fw_motor_duty_cycle * 500 + 500));  // Send to the right friction wheel
}

/**
  * @brief initialization
  */
void ShootInterface::shoot_calc_init() {

    palSetPadMode(GPIOI, 5, PAL_MODE_ALTERNATE(3));

    palSetPadMode(GPIOI, 6, PAL_MODE_ALTERNATE(3));

    pwmStart(&PWMD8, &friction_wheels_pwmcfg);

}