//
// Created by 钱晨 on 2019-05-18.
//

#ifndef META_INFANTRY_THREAD_SHOOT_H
#define META_INFANTRY_THREAD_SHOOT_H

/**
 * @name ShootThread
 * @brief Thread to control shooter
 * @pre Remote interpreter starts receiving
 * @pre GimbalInterface(Shoot) get properly init()
 * @pre GimbalThread starts
 */

 class ShootThread : public chibios_rt::BaseStaticThread<1024>{

     static constexpr unsigned int SHOOT_THREAD_INTERVAL = 5; // PID calculation interval [ms]

     static constexpr  float  COMMON_SHOOT_SPEED = 5;

     bool pc_right_pressed = false; // local variable to control one click of right button of mouse

     void main() final {

         setName("shoot");

         bool bullet_full = false;
         float plate_target_angle = Shoot::feedback[3].actual_angle;
         float bullet_target_angle = 180.0f;

         Shoot::change_pid_params(GIMBAL_PID_BULLET_LOADER_A2V_PARAMS, GIMBAL_PID_BULLET_LOADER_V2I_PARAMS, GIMBAL_PID_BULLET_PLATE_A2V_PARAMS, GIMBAL_PID_BULLET_PLATE_V2I_PARAMS);
         while (!shouldTerminate()) {

             bullet_full = (bool) palReadPad(GPIOE,GPIOB_PIN54); // check if bullet is full or not, automatically load. Currently use J2 pin. TODO: determined the PIN to use.
             if (plate_target_angle > 360.0f && Shoot::feedback[3].actual_angle < plate_target_angle -360.0f) plate_target_angle -= 360.0f; //get the correct angle first
             if(!bullet_full){
                 if(plate_target_angle - Shoot::feedback[3].actual_angle < 3.0f) plate_target_angle += 36.0f;
             }
             Shoot::calc_plate(Shoot::feedback[3].actual_velocity, plate_target_angle);

             if (bullet_target_angle > 360.0f && Shoot::feedback[2].actual_angle < bullet_target_angle -360.0f) bullet_target_angle -= 360.0f; //get the correct angle first
             if (!StateHandler::remoteDisconnected() && !StateHandler::gimbalSeriousErrorOccured()){
                 if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP){

                     // Friction wheels
                     Shoot::set_friction_wheels(0);

                     // Bullet loader motor
                     if (Remote::rc.ch1 > 0.5) {
                         if (Shoot::fw_duty_cycle == 0) {
                             Shoot::set_friction_wheels(GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE);
                             sleep(TIME_I2MS(500));
                         }
                         if(bullet_target_angle - Shoot::feedback[2].actual_angle < 3.0f) bullet_target_angle += 72.0f;
                     }

                 } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

                     // Friction wheels
                     Shoot::set_friction_wheels(GIMBAL_REMOTE_FRICTION_WHEEL_DUTY_CYCLE);

                     // Bullet loader motor
                     if (Remote::rc.ch3 < -0.1) {
                         if (Shoot::fw_duty_cycle == 0) {
                             Shoot::set_friction_wheels(GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE);
                             sleep(TIME_I2MS(500));
                         }
                         if(bullet_target_angle - Shoot::feedback[2].actual_angle < 3.0f) bullet_target_angle += 72.0f;
                     }

                 } else if (Remote::rc.s1 == Remote::S_DOWN) { // PC control mode

                     // Bullet loader motor
                     if (Remote::mouse.press_left) {

                         if (Shoot::fw_duty_cycle == 0) {
                             Shoot::set_friction_wheels(GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE);
                             sleep(TIME_I2MS(500));
                         }
                         if(bullet_target_angle - Shoot::feedback[2].actual_angle < 3.0f) bullet_target_angle += 72.0f;
                     }

                     // Friction wheels
                     if (Remote::mouse.press_right) {
                         if (!pc_right_pressed) {
                             if (Shoot::fw_duty_cycle == 0) {
                                 Shoot::set_friction_wheels(GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE);
                             } else {
                                 Shoot::set_friction_wheels(0);
                             }
                             pc_right_pressed = true;
                         }
                     } else {
                         if (pc_right_pressed) {
                             pc_right_pressed = false;
                         }
                     }
                 } else {

                     Shoot::set_friction_wheels(0);
                     Shoot::target_current[Shoot::BULLET] = 0;
                     Shoot::target_current[Shoot::PLATE] = 0;

                 }

             } else {
                 Shoot::set_friction_wheels(0);
                 Shoot::target_current[Shoot::BULLET] = 0;
                 Shoot::target_current[Shoot::PLATE] = 0;
             }
             Shoot::calc_bullet(Shoot::feedback[2].actual_velocity, bullet_target_angle);
             sleep(TIME_MS2I(SHOOT_THREAD_INTERVAL));
         }
     }
 };
#endif //META_INFANTRY_THREAD_SHOOT_HPP

