//
// Created by 钱晨 on 2019-07-03.
//

#include "hero_shoot_logic.h"

#include "shell.h"
#include "referee_interface.h"
#include "shoot_scheduler.h"
#include "string.h"

float HeroShootLG::loader_angle_per_bullet = 0.0f;
float HeroShootLG::plate_angle_per_bullet = 0.0f;
float HeroShootLG::loader_target_angle = 0.0f;
float HeroShootLG::plate_target_angle = 0.0f;
int  HeroShootLG::plate_runtime = 0;
bool HeroShootLG::loaded_bullet[4] = {false, false, false, false};
// loaded_bullet[0] is the closest bullet place to the friction wheel.
int HeroShootLG::load_bullet_count = 0;
HeroShootLG::loader_state_t HeroShootLG::loaderState = STOP;
HeroShootLG::loader_state_t HeroShootLG::plateState = STOP;

HeroShootLG::AutoLoaderThread HeroShootLG::autoLoader;
HeroShootLG::StuckDetectorThread HeroShootLG::stuckDetector;
HeroShootLG::plateLoadAttempt HeroShootLG::PlateLoadAttempt;

void HeroShootLG::init(float loader_angle_per_bullet_, float plate_angle_per_bullet_,
                       tprio_t stuck_detector_thread_prio, tprio_t auto_loader_thread_prio) {

    // Initialize pa
    // rameters
    loader_angle_per_bullet = loader_angle_per_bullet_;
    plate_angle_per_bullet = plate_angle_per_bullet_;

    // Clear the bullet angle
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::reset_plate_accumulated_angle();

    stuckDetector.start(stuck_detector_thread_prio);
    autoLoader.start(auto_loader_thread_prio);
}

void HeroShootLG::shoot() {

    if (loaderState == STOP) {
        ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
        if (loaded_bullet[0] && loaded_bullet[1]) {    // if the next bullet place is loaded

            loader_target_angle += loader_angle_per_bullet;
            ShootSKD::set_loader_target_angle(loader_target_angle);
            loaderState = LOADING;

            // Update the status
            loaded_bullet[0] = loaded_bullet[1];
            loaded_bullet[1] = loaded_bullet[2];
            loaded_bullet[2] = loaded_bullet[3];
            loaded_bullet[3] = false;

        } else if (loaded_bullet[0] && !loaded_bullet[1]) {    // if the next bullet place is empty

            loader_target_angle += (2 * loader_angle_per_bullet);
            ShootSKD::set_loader_target_angle(loader_target_angle);
            loaderState = LOADING;

            // Update the status
            loaded_bullet[0] = loaded_bullet[2];
            loaded_bullet[1] = loaded_bullet[3];
            loaded_bullet[3] = false;

        } else if (!loaded_bullet[0]) {
            loader_target_angle += (2 * loader_angle_per_bullet);
            ShootSKD::set_loader_target_angle(loader_target_angle);
            loaderState = LOADING;

            // Update the status
            loaded_bullet[0] = loaded_bullet[2];
            loaded_bullet[1] = loaded_bullet[3];
            loaded_bullet[3] = false;
            loaded_bullet[2] = (bool) palReadPad(GPIOE, GPIOE_PIN5);
        }
    }
}

void HeroShootLG::set_friction_wheels(float duty_cycle) {
    ShootSKD::set_friction_wheels(duty_cycle);
    Referee::set_client_light(USER_CLIENT_FW_STATE_LIGHT, (duty_cycle != 0));
    // Sending client data will be complete by higher level thread
}

float HeroShootLG::get_friction_wheels_duty_cycle() {
    return ShootSKD::get_friction_wheels_duty_cycle();
}

void HeroShootLG::force_stop() {
    ShootSKD::set_mode(ShootSKD::FORCED_RELAX_MODE);
}

void HeroShootLG::StuckDetectorThread::main() {
    setName("Stuck_Detector");
    while (!shouldTerminate()) {
        if (loaderState == LOADING &&
            ShootSKD::get_loader_target_current() > LOADER_STUCK_THRESHOLD_CURRENT &&
            ShootSKD::get_loader_actual_velocity() < LOADER_STUCK_THRESHOLD_VELOCITY) {
            loaderState = STUCK;
            ShootSKD::set_loader_target_angle(
                    ShootSKD::get_loader_accumulated_angle() - 20.0f);  // Back up to ample space
        }
        if(plateState == LOADING &&
           ShootSKD::get_plate_target_current() > PLATE_STUCK_THRESHOLD_CURRENT &&
           ShootSKD::get_plate_actual_velocity() < PLATE_STUCK_THRESHOLD_VELOCITY &&
           load_bullet_count != 0) {
            plateState = STUCK;
            ShootSKD::set_plate_target_angle(ShootSKD::get_plate_accumulated_angle() - 10.0f);  // Back up to ample space
        }
        if (loaderState == STUCK || plateState == STUCK) {

            // Give some time to let the loaders to reverse.
            sleep(TIME_MS2I(STUCK_REVERSE_TIME));

            // Update the loaders' states & reset the target angle.
            if(loaderState == STUCK) {
                loaderState = LOADING;
                ShootSKD::set_loader_target_angle(loader_target_angle);
            }
            if(plateState == STUCK) {
                plateState = LOADING;
                ShootSKD::set_plate_target_angle(plate_target_angle);
            }

        }
        sleep(TIME_MS2I(STUCK_DETECTOR_THREAD_INTERVAL));
    }
}

void HeroShootLG::AutoLoaderThread::main() {
    setName("Automation");
    PlateLoadAttempt.wait_time = SYSTIME;
    PlateLoadAttempt.attempt_number = 0;
    PlateLoadAttempt.task_status = LOAD_SUCCESS;
    int ball_in_tunnel = 0;
    int ball_in_loader = 0;

    bool PlateMouthStatus[2] = {false, false};
    bool LoaderMouthStatus[2] = {false, false};

    while (!shouldTerminate()) {

        // Update the MouthStates
        PlateMouthStatus[0] = PlateMouthStatus[1];
        PlateMouthStatus[1] = (bool) palReadPad(GPIOE, GPIOE_PIN5);

        LoaderMouthStatus[0] = LoaderMouthStatus[1];
        LoaderMouthStatus[1] = (bool) palReadPad(GPIOE, GPIOE_PIN4);

        // Update the loader State
        if (loader_target_angle - ShootSKD::get_loader_accumulated_angle() > 5.0f && loaderState != STUCK) {
            loaderState = LOADING;
        } else if (loader_target_angle - ShootSKD::get_loader_accumulated_angle() < 5.0f && loaderState != STUCK) {
            loaderState = STOP;
        }

        // Update the plate State.
        if (plate_target_angle - ShootSKD::get_plate_accumulated_angle() > 3.0f && plateState != STUCK) {
            plateState = LOADING;
        } else if (plate_target_angle - ShootSKD::get_plate_accumulated_angle() < 3.0f && plateState != STUCK) {
            plateState = STOP;
        }

        // Update sequence by detecting the last bullet place
        loaded_bullet[2] = (bool) palReadPad(GPIOE, GPIOE_PIN4);

        // When loaded_bullet[2] is true and an extra loading bullet is detected
        if (loaded_bullet[2] && (bool) palReadPad(GPIOE, GPIOE_PIN5)) {
            loaded_bullet[3] = true;
        }

        // loader load automatically.
        if (!loaded_bullet[0] && loaded_bullet[2] && loaderState == STOP) {
            ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
            // get rid of empty bullet place
            loader_target_angle += loader_angle_per_bullet;
            ShootSKD::set_loader_target_angle(loader_target_angle);
            loaderState = LOADING;
            loaded_bullet[0] = loaded_bullet[1];
            loaded_bullet[1] = loaded_bullet[2];
            loaded_bullet[2] = loaded_bullet[3];
            loaded_bullet[3] = false;
            // Maybe no need to update the loaded_bullet[2] as the loaded_bullet[2] will refresh real timely.
        }

        /**             Plate Auto Load*/

        // Set for load numbers.
        if (plateState == STOP){
            // Situation I: Previous Load Success and now need to load
            if (PlateLoadAttempt.task_status == LOAD_SUCCESS){
                // If the last place is false
                if(loaded_bullet[2] == false) {

                    PlateLoadAttempt.task_status = LOAD_RUNNING;

                    // Situations for load two bullets:
                    // true, false, false (shoot will turn 144 degree)
                    // false, false, false
                    if (loaded_bullet[1] == false && loaded_bullet[2] == false){
                        PlateLoadAttempt.attempt_number = 2;
                        memcpy(PlateLoadAttempt.bullet_status, loaded_bullet, sizeof(loaded_bullet));
                    } else if (loaded_bullet[1] == true && loaded_bullet[2] == false) {
                        PlateLoadAttempt.attempt_number = 1;
                        memcpy(PlateLoadAttempt.bullet_status, loaded_bullet, sizeof(loaded_bullet));
                    }
                }
            }
            // Situation II: Previous Load Failed and need to reload. (Ball has slipped away.)
            else if (PlateLoadAttempt.task_status == LOAD_WAITING){
                // wait_time has beyond the tolerable time.
                if(SYSTIME - PlateLoadAttempt.wait_time > 500) {
                    PlateLoadAttempt.attempt_number = PlateLoadAttempt.attempt_number - ball_in_tunnel;
                    PlateLoadAttempt.task_status = LOAD_RUNNING;
                    ball_in_tunnel = 0;
                }
            }
            // Situation III: Load is running, handle the task
            else if (PlateLoadAttempt.task_status == LOAD_RUNNING){
                ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
                if(ball_in_tunnel + ball_in_loader < PlateLoadAttempt.attempt_number){
                    ShootSKD::set_plate_target_angle(plate_target_angle);
                } else if (ball_in_tunnel + ball_in_loader >= PlateLoadAttempt.attempt_number) {
                    PlateLoadAttempt.task_status = LOAD_WAITING;
                }
            }
        }

        // Update the ball in tunnel
        if (PlateMouthStatus[0] && !PlateMouthStatus[1]) {
            ball_in_tunnel += 1;
        }
        if (!LoaderMouthStatus[0] && LoaderMouthStatus[1]) {
            ball_in_tunnel -= 1;
            ball_in_loader += 1;
            PlateLoadAttempt.wait_time = SYSTIME;
        }
        // Once the a bullet was in, refresh the wait time.
        if (PlateLoadAttempt.task_status == LOAD_WAITING && (bool)palReadPad(GPIOE, GPIOE_PIN4)) PlateLoadAttempt.wait_time = SYSTIME;

        // Update wait time.


        // Check load success or not.
        if (ball_in_loader >= PlateLoadAttempt.attempt_number) {
            PlateLoadAttempt.task_status = LOAD_SUCCESS;
        }

        sleep(TIME_MS2I(AUTO_LOADER_THREAD_INTERVAL));
    }

}