//
// Created by liuzikai on 2018-12-30.
//

#include "gimbal_feedback_module.h"

void GimbalFeedbackModule::main() {

    setName("gimbal_fb");

    while (!shouldTerminate()) {
        chSysLock();
//        float yaw_target_angle = (ptr_yaw_target_angle ? *ptr_yaw_target_angle : 0);
//        float yaw_target_velocity = (ptr_yaw_target_velocity ? *ptr_yaw_target_velocity : 0);
//        int yaw_target_current = (ptr_yaw_target_current ? *ptr_yaw_target_current : 0);
//        Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
//                TIME_I2MS(chibios_rt::System::getTime()),
//                GimbalInterface::yaw.actual_angle, yaw_target_angle,
//                GimbalInterface::yaw.angular_velocity, yaw_target_velocity,
//                GimbalInterface::yaw.actual_current, yaw_target_current);
////        Shell::printf("yaw round = %d" SHELL_NEWLINE_STR,
////                      GimbalInterface::yaw.round_count);
//        float pitch_target_angle = (ptr_pitch_target_angle ? *ptr_pitch_target_angle : 0);
//        float pitch_target_velocity = (ptr_pitch_target_velocity ? *ptr_pitch_target_velocity : 0);
//        int pitch_target_current = (ptr_pitch_target_current ? *ptr_pitch_target_current : 0);
//        Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
//                TIME_I2MS(chibios_rt::System::getTime()),
//                GimbalInterface::pitch.actual_angle, pitch_target_angle,
//                GimbalInterface::pitch.angular_velocity, pitch_target_velocity,
//                GimbalInterface::pitch.actual_current, pitch_target_current);
////        Shell::printf("pitch round = %d" SHELL_NEWLINE_STR,
////                      GimbalInterface::pitch.round_count);
        chSysUnlock();
        sleep(TIME_MS2I(feedback_interval));
    }

}