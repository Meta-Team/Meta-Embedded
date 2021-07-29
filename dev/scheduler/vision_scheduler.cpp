//
// Created by liuzikai on 7/29/21.
//

#include "vision_scheduler.h"
#include "vision_interface.h"
#include "ahrs_math.hpp"
#include "trajectory_calculator.hpp"

EVENTSOURCE_DECL(VisionSKD::gimbal_updated_event);
EVENTSOURCE_DECL(VisionSKD::shoot_time_updated_event);
float VisionSKD::bullet_speed = 15;
time_msecs_t VisionSKD::basic_gimbal_delay = 0;
PositionKalmanFilter VisionSKD::armor_ypd[3];
constexpr float VisionSKD::ARMOR_Q_POSITION[3];
constexpr float VisionSKD::ARMOR_Q_VELOCITY[3];
constexpr float VisionSKD::ARMOR_R_POSITION[3];
uint16_t VisionSKD::last_frame_timestamp = 0;
time_msecs_t VisionSKD::last_detected_time = 0;
time_msecs_t VisionSKD::last_compute_time = 0;
time_msecs_t VisionSKD::last_vision_command_time = 0;
bool VisionSKD::can_reach_target = false;
float VisionSKD::bullet_flight_time = 0;
float VisionSKD::latest_target_yaw = 0;
float VisionSKD::latest_target_pitch = 0;
time_msecs_t VisionSKD::expected_shoot_time = 0;
int VisionSKD::expected_shoot_after_periods = 0;
float VisionSKD::IMAGE_TO_USER_SCALE = 0.59;
int VisionSKD::IMAGE_TO_USER_OFFSET_X = 770;
int VisionSKD::IMAGE_TO_USER_OFFSET_Y = 400;
uint32_t VisionSKD::user_view_x = 0;
uint32_t VisionSKD::user_view_y = 0;
VisionSKD::CalculationThread VisionSKD::calculation_thread;

void VisionSKD::start(time_msecs_t basic_gimbal_delay_, tprio_t thread_prio) {
    for (int i = 0; i < 3; i++) {
        armor_ypd[i].set_Q_position(ARMOR_Q_POSITION[i]);
        armor_ypd[i].set_Q_velocity(ARMOR_Q_VELOCITY[i]);
        armor_ypd[i].set_R_position(ARMOR_R_POSITION[i]);
    }
    basic_gimbal_delay = basic_gimbal_delay_;

    calculation_thread.start(thread_prio);
}

bool VisionSKD::is_detected() {
    return WITHIN_RECENT_TIME(last_detected_time, DETECTION_LOSE_TIME);
}

bool VisionSKD::get_gimbal_target_angles(float &yaw, float &pitch) {
    if (WITHIN_RECENT_TIME(last_detected_time, DETECTION_LOSE_TIME)) {
        if (!can_reach_target) {
            return false;
        } else {
            yaw = latest_target_yaw;
            pitch = latest_target_pitch;
            return true;
        }
    } else {
        return false;
    }
}

void VisionSKD::get_user_view_points(uint32_t &x, uint32_t &y) {
    x = user_view_x;
    y = user_view_y;
}

void VisionSKD::CalculationThread::main() {
    setName("VisionSKD");

    VisionIF::vision_command_t command{};
    float new_armor_yaw, new_armor_pitch;

    while (!shouldTerminate()) {

        if (WITHIN_RECENT_TIME(VisionIF::get_last_valid_update_time(), 2000)) {

            bool new_command_received = false;
            if (VisionIF::get_last_valid_update_time() != last_vision_command_time) {
                last_detected_time = last_vision_command_time = VisionIF::get_last_valid_update_time();
                VisionIF::get_latest_valid_command(command, new_armor_yaw, new_armor_pitch);
                new_command_received = true;
            }

            auto time_delta = (float) (SYSTIME - last_compute_time);

            if (new_command_received) {

                // Update Kalman filters with received command
                Vector3D ypd = Vector3D{new_armor_yaw, new_armor_pitch, (float) command.dist};
                Vector3D last_ypd = Vector3D{armor_ypd[0].get_position(),
                                             armor_ypd[1].get_position(),
                                             armor_ypd[2].get_position()};

                if ((int16_t) (command.time_stamp - last_frame_timestamp) < (int16_t) POSITION_RELOAD_TIME &&
                    (ypd - last_ypd).norm() < SINGLE_ARMOR_2D_OFFSET_PER_FRAME) {
                    armor_ypd[0].predict_update(new_armor_yaw, time_delta);
                    armor_ypd[1].predict_update(new_armor_pitch, time_delta);
                    armor_ypd[2].predict_update(command.dist, time_delta);
                } else {
                    armor_ypd[0].reload_position(ypd.x);
                    armor_ypd[1].reload_position(ypd.y);
                    armor_ypd[2].reload_position(ypd.z);
                }
                last_compute_time = SYSTIME;
                last_frame_timestamp = command.time_stamp;

            } else {

                // Update Kalman filters with only prediction
//                for (auto &filter : armor_ypd) filter.predict(time_delta);

                sleep(TIME_MS2I(2));
                continue;

            }


            float yaw = armor_ypd[0].get_position();
            float pitch = armor_ypd[1].get_position();
            float dist = armor_ypd[2].get_position();

            // Predict movement
            {
                float rough_flight_time = dist / bullet_speed;  // not accurate, but probably enough
                float predict_forward_amount = (float) basic_gimbal_delay + rough_flight_time;
                yaw += armor_ypd[0].get_velocity() * predict_forward_amount;
            }

            // Compensate for gravity
            can_reach_target = Trajectory::compensate_for_gravity(pitch, dist, bullet_speed, bullet_flight_time);

            if (can_reach_target) {

                // Issue new gimbal command
                latest_target_yaw = yaw;
                latest_target_pitch = pitch;
                chEvtBroadcast(&gimbal_updated_event);  // we are still in I-Lock state

                // Predict time to shoot
                /*if (command.flag & TOP_KILLER_TRIGGERED) {

                    // Calculate using time_msecs_t so that we don't need to care about wrap around
                    time_msecs_t shoot_time = last_update_time
                                              + (time_msecs_t) command.remaining_time_to_target
                                              - (time_msecs_t) flight_time_to_target
                                              - (time_msecs_t) measured_shoot_delay.get();

                    // Compensate for one or more periods until we can catch up expected_shoot_time
                    expected_shoot_after_periods = 0;
                    while (true) {
                        // Minus using time_msecs_t so that we don't need to care about wrap around
                        auto time_delta = (int32_t) (shoot_time - now);  // compare by casting to signed
                        if (time_delta < 0) {
                            expected_shoot_after_periods++;
                            shoot_time += command.period;
                        } else {
                            break;
                        }
                    }

                    // Issue shoot command
                    expected_shoot_time = (time_msecs_t) shoot_time;
                    chEvtBroadcast(&shoot_time_updated_event);  // we are still in I-Lock state

                } else {
                    expected_shoot_time = 0;
                }*/

                // Update user view
                {
                    user_view_x = IMAGE_TO_USER_OFFSET_X + command.imageX * IMAGE_TO_USER_SCALE;
                    user_view_y = 1080 - (IMAGE_TO_USER_OFFSET_Y + command.imageY * IMAGE_TO_USER_SCALE);
                }
            }
        }

        sleep(TIME_MS2I(2));
    }
}

const Shell::Command VisionSKD::shell_commands[] = {
        {"_v",           nullptr,                                  VisionSKD::cmdInfo,               nullptr},
        {"_v_enable_fb", "Channel/All Feedback{Disabled,Enabled}", VisionSKD::cmdEnableFeedback,     nullptr},
        {"_v_user_view", "[Scale] [X_Offset] [Y_Offset]",          VisionSKD::cmd_user_view_setting, nullptr},
        {nullptr,        nullptr,                                  nullptr,                          nullptr}
};

DEF_SHELL_CMD_START(VisionSKD::cmdInfo)
    Shell::printf("_v:VisionSKD" ENDL);
    Shell::printf("_v/Armor:X{Angle,Velocity} Y{Angle,Velocity} Z{Angle,Velocity}" ENDL);
    return true;
DEF_SHELL_CMD_END

static bool feedbackEnabled[1] = {false};

void VisionSKD::cmd_feedback(void *) {

    if (feedbackEnabled[0]) {
        Shell::printf("_v0 %.2f %.2f %.2f %.2f %d %d" ENDL,
                      armor_ypd[0].get_position(), armor_ypd[0].get_velocity() * 1000,
                      armor_ypd[1].get_position(), armor_ypd[1].get_velocity() * 1000,
                      armor_ypd[2].get_position(), armor_ypd[2].get_velocity() * 1000);
    }

}

DEF_SHELL_CMD_START(VisionSKD::cmdEnableFeedback)
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, sizeof(feedbackEnabled) / sizeof(*feedbackEnabled), id, enabled))
        return false;
    if (id == -1) {
        for (bool &e : feedbackEnabled) e = enabled;
    } else {
        feedbackEnabled[id] = enabled;
    }
    return true;
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(VisionSKD::cmd_user_view_setting)
    if (argc == 0) {
        Shell::printf("_v_user_view %f %d %d" ENDL, IMAGE_TO_USER_SCALE, IMAGE_TO_USER_OFFSET_X,
                      IMAGE_TO_USER_OFFSET_Y);
        return true;
    } else if (argc == 3) {
        IMAGE_TO_USER_SCALE = Shell::atof(argv[0]);
        IMAGE_TO_USER_OFFSET_X = Shell::atoi(argv[1]);
        IMAGE_TO_USER_OFFSET_Y = Shell::atoi(argv[2]);
        return true;
    } else {
        return false;
    }
DEF_SHELL_CMD_END