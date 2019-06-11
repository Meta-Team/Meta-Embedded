//
// Created by on 2019-05-18.
//

#ifndef META_INFANTRY_THREAD_GIMBAL_H
#define META_INFANTRY_THREAD_GIMBAL_H

/**
 * @name GimbalThread
 * @brief Thread to control gimbal yaw and pitch
 * @pre Remote interpreter starts receiving
 * @pre MPU 6500 starts updating
 * @pre Gimbal Interface(Gimbal) get properly init()
 */
 class GimbalThread : public chibios_rt::BaseStaticThread<1024> {

     static constexpr unsigned int GIMBAL_THREAD_INTERVAL = 1; // PID calculation interval [ms]

     static constexpr float PC_YAW_SPEED_RATIO = 54000;     // rotation speed when mouse moves fastest [degree/s]
     static constexpr float PC_YAW_ANGLE_LIMITATION = 60;   // maximum range (both CW and CCW) [degree]

     static constexpr float PC_PITCH_SPEED_RATIO = 12000;   // rotation speed when mouse moves fastest [degree/s]
     static constexpr float PC_PITCH_ANGLE_LIMITATION = 30; // maximum range (both up and down) [degree]

     float pc_yaw_current_target_angle = 0;
     float pc_pitch_current_target_angle = 0;

     void main() final {

         setName("gimbal");

         Gimbal::change_pid_params(GIMBAL_PID_YAW_A2V_PARAMS,
                                   GIMBAL_PID_YAW_V2I_PARAMS,
                                   GIMBAL_PID_PITCH_A2V_PARAMS,
                                   GIMBAL_PID_PITCH_V2I_PARAMS);

         while(!shouldTerminate()) {

             if (!StateHandler::remoteDisconnected() && !StateHandler::gimbalSeriousErrorOccured()) {

                 if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

                     Gimbal::calc_gimbal(GIMBAL_YAW_ACTUAL_VELOCITY, GIMBAL_PITCH_ACTUAL_VELOCITY,
                                         -Remote::rc.ch0 * 45, // Yaw   Target angle
                                         0                     // Pitch Target angle
                     );

                 } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {
                     Gimbal::calc_gimbal(GIMBAL_YAW_ACTUAL_VELOCITY, GIMBAL_PITCH_ACTUAL_VELOCITY,
                                         -Remote::rc.ch0 * 45,  // Yaw  target angle
                                         -Remote::rc.ch1 * 20  // Pitch target angle
                     );

                 } else if (Remote::rc.s1 == Remote::S_DOWN) {

                     if (Remote::key.v) {
                         pc_yaw_current_target_angle = pc_pitch_current_target_angle = 0;
                     } else {
                         pc_yaw_current_target_angle +=
                                 -Remote::mouse.x * (PC_YAW_SPEED_RATIO * (GIMBAL_THREAD_INTERVAL / 1000.0f));
                         pc_pitch_current_target_angle +=
                                 -Remote::mouse.y * (PC_PITCH_SPEED_RATIO * (GIMBAL_THREAD_INTERVAL / 1000.0f));
                     }

                     ABS_CROP(pc_yaw_current_target_angle, PC_YAW_ANGLE_LIMITATION);
                     ABS_CROP(pc_pitch_current_target_angle, PC_PITCH_ANGLE_LIMITATION);

                     Gimbal::calc_gimbal(GIMBAL_YAW_ACTUAL_VELOCITY, GIMBAL_PITCH_ACTUAL_VELOCITY,
                                         pc_yaw_current_target_angle,  // Yaw   target angle
                                         pc_pitch_current_target_angle // Pitch target angle
                     );

                 } else {

                     Gimbal::target_current[Gimbal::YAW] = Gimbal::target_current[Gimbal::PITCH] = 0;

                 }

             } else {

                 Gimbal::target_current[Gimbal::YAW] = Gimbal::target_current[Gimbal::PITCH] = 0;

             }

             GimbalInterface::send_gimbal_currents();

             sleep(TIME_MS2I((GIMBAL_THREAD_INTERVAL)));
         }
     }
 };
#endif //META_INFANTRY_THREAD_GIMBAL_HPP
