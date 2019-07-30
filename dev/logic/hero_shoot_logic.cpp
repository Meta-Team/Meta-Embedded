//
// Created by 钱晨 on 2019-07-03.
//

#include "hero_shoot_logic.h"

#include "shell.h"
#include "referee_interface.h"
#include "shoot_scheduler.h"
#include "math.h"

float HeroShootLG::loader_angle_per_bullet = 0.0f;
float HeroShootLG::plate_angle_per_bullet = 0.0f;
float HeroShootLG::loader_target_angle = 0.0f;
float HeroShootLG::plate_target_angle = 0.0f;
int  HeroShootLG::plate_runtime = 0;
float HeroShootLG::plate_angle_increment = 0.0f;
float HeroShootLG::loader_angle_increment = 0.0f;

HeroShootLG::bullet_type_t HeroShootLG::bulletType = TRANSPARENT;

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

    // Initialize parameters
    loader_angle_per_bullet = loader_angle_per_bullet_;
    plate_angle_per_bullet = plate_angle_per_bullet_;

    // Clear the bullet angle
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::reset_plate_accumulated_angle();

    stuckDetector.start(stuck_detector_thread_prio);
    autoLoader.start(auto_loader_thread_prio);
}

bool HeroShootLG::get_loader_mouth_status() {
    if (bulletType == TRANSPARENT) {
        return !(bool) palReadPad(GPIOF, GPIOF_PIN0);
    } else if (bulletType == WHITE){
        return  (bool) palReadPad(GPIOF, GPIOF_PIN0);
    }
}

bool HeroShootLG::get_plate_mouth_status() {
    if (bulletType == TRANSPARENT) {
        return !(bool) palReadPad(GPIOF, GPIOF_PIN1);
    } else if (bulletType == WHITE) {
        return  (bool) palReadPad(GPIOF, GPIOF_PIN1);
    }
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
            loaded_bullet[2] = false;

            loader_angle_increment = loader_angle_per_bullet;
        } else if (loaded_bullet[0] && !loaded_bullet[1]) {

            loader_target_angle += (2 * loader_angle_per_bullet);
            ShootSKD::set_loader_target_angle(loader_target_angle);
            loaderState = LOADING;

            // Update the status
            loaded_bullet[0] = loaded_bullet[2];
            loaded_bullet[1] = false; // this status will be updated in Automation thread.
            loaded_bullet[2] = false;

            loader_angle_increment = 2 * loader_angle_per_bullet;
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
    float loader_angle[4] = {0.0f,0.0f,0.0f,0.0f};
    int last_loader_update_time = SYSTIME;
    while (!shouldTerminate()) {
        if(SYSTIME - last_loader_update_time > 200) {
            loader_angle[0] = loader_angle[1];
            loader_angle[1] = loader_angle[2];
            loader_angle[2] = loader_angle[3];
            loader_angle[3] = ShootSKD::get_loader_accumulated_angle();
            last_loader_update_time= SYSTIME;
        }
        if (loaderState == LOADING &&
            fabs(loader_angle[0] - loader_angle[3]) < 0.5f &&
            fabs(loader_angle[1] - loader_angle[3]) < 0.5f &&
            fabs(loader_angle[2] - loader_angle[3]) < 0.5f) {
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
        /***-----------1.Update-the-status-----------***/

        // I.Detect the Mouths
        //   Loader Mouth
        LoaderMouthStatus[0] = LoaderMouthStatus[1];
        LoaderMouthStatus[1] = get_loader_mouth_status();
        //   Plate Mouth
        PlateMouthStatus[0] = PlateMouthStatus[1];
        PlateMouthStatus[1] = get_plate_mouth_status();

        // II.The last place of loader
        loaded_bullet[2] = get_loader_mouth_status();

        // III.Detect the extra bullet in loading. (Should Only enabled when load 2 places, to detect the extra bullet)
        //     When a bullet passed by while loading, it means the extra bullet was successfully loaded.

        if (loaderState == LOADING && loader_angle_increment == 144.0f &&
            !LoaderMouthStatus[0] && LoaderMouthStatus[1]) {
            // Only add the when the previous bullet has passed.
            if (loader_target_angle - ShootSKD::get_loader_accumulated_angle() <114.0f) {
                if(!loaded_bullet[1]) loaded_bullet[1] = true;
            }
        }

        // IV. Update the loaders' states

        //     Loader
        if(loaderState != STUCK && loader_target_angle - ShootSKD::get_loader_accumulated_angle() <= 5.0f) {
            loaderState = STOP;
        } else if (loaderState != STUCK && loader_target_angle - ShootSKD::get_loader_accumulated_angle() >5.0f ){
            loaderState = LOADING;
        }
        //     Plate
        if(plateState != STUCK && plate_target_angle - ShootSKD::get_plate_accumulated_angle() <= 3.0f) {
            plateState = STOP;
        } else if (plateState != STUCK && plate_target_angle - ShootSKD::get_plate_accumulated_angle() > 3.0f) {
            plateState = LOADING;
        }

        // V. Update the ball in tunnel and ball in loader.
        //    Ball in loader
        if (!LoaderMouthStatus[0] && LoaderMouthStatus[1]) {
            ball_in_loader += 1;
            ball_in_tunnel -= 1;
        }
        //    Ball in tunnel
        if (PlateMouthStatus[0] && !PlateMouthStatus[1]) {
            ball_in_tunnel += 1;
        }

        /***-----------2.Auto-Load-Loaders-----------***/

        // I. Loader Auto Load
        //    Read the pin and auto Load.
        if (loaderState == STOP) {
            if (get_loader_mouth_status() && !loaded_bullet[0]) {
                ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
                loader_target_angle += loader_angle_per_bullet;
                ShootSKD::set_loader_target_angle(loader_target_angle);
                loaderState = LOADING;

                // Update the status.
                loaded_bullet[0] = loaded_bullet[1];
                loaded_bullet[1] = loaded_bullet[2];
                loaded_bullet[2] =  get_loader_mouth_status();

                loader_angle_increment = loader_angle_per_bullet;
            }
        }
        // II.Plate Auto Load
        if (plateState == STOP) {
            if (get_loader_mouth_status() && PlateLoadAttempt.task_status == LOAD_SUCCESS) {
                if (loaded_bullet[0] && loaded_bullet[1] && !loaded_bullet[2]) {
                    PlateLoadAttempt.task_status = LOAD_RUNNING;
                    PlateLoadAttempt.attempt_number = 1;
                    ball_in_loader = 0;
                    ball_in_tunnel = 0;
                } else if (loaded_bullet[0] && !loaded_bullet[1] && !loaded_bullet[2]) {
                    PlateLoadAttempt.task_status = LOAD_RUNNING;
                    PlateLoadAttempt.attempt_number = 2;
                    ball_in_loader = 0;
                    ball_in_tunnel = 0;
                } else if (!loaded_bullet[0] && !loaded_bullet[1] && !loaded_bullet[2]) {
                    PlateLoadAttempt.task_status = LOAD_RUNNING;
                    PlateLoadAttempt.attempt_number = 3;
                    ball_in_loader = 0;
                    ball_in_tunnel = 0;
                }
            }
            if (PlateLoadAttempt.task_status == LOAD_RUNNING) {
                if (ball_in_loader + ball_in_tunnel < PlateLoadAttempt.attempt_number) {
                    // if the ball in tunnel was less than the number we set, just load.
                    ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
                    plate_target_angle += plate_angle_per_bullet;
                    plate_angle_increment = plate_angle_per_bullet;
                    ShootSKD::set_plate_target_angle(plate_target_angle);
                } else if (ball_in_loader + ball_in_tunnel > PlateLoadAttempt.attempt_number) {
                    PlateLoadAttempt.task_status = LOAD_WAITING;
                    PlateLoadAttempt.wait_time = SYSTIME;
                }
            }
            if (PlateLoadAttempt.task_status == LOAD_WAITING) {
                if (ball_in_loader >= PlateLoadAttempt.attempt_number) {
                    PlateLoadAttempt.task_status = LOAD_SUCCESS;
                }
                if (SYSTIME - PlateLoadAttempt.wait_time > 1500) {
                    PlateLoadAttempt.task_status = LOAD_SUCCESS;
                } // After Replace the tunnel, no bullet would leave.
            }
        }
        sleep(TIME_MS2I(AUTO_LOADER_THREAD_INTERVAL));
    }

}