//
// Created by 钱晨 on 2019-07-03.
//

#include "hero_shoot_logic.h"

float HeroShootLG::loader_angle_per_bullet = 0.0f;
float HeroShootLG::plate_angle_per_bullet = 0.0f;
float HeroShootLG::loader_target_angle = 0.0f;
float HeroShootLG::plate_target_angle = 0.0f;
bool HeroShootLG::loaded_bullet[4] = {false, false, false, false};
    // loaded_bullet[0] is the closest bullet place to the friction wheel.
int HeroShootLG::load_bullet_count = 0;
HeroShootLG::loader_state_t HeroShootLG::loaderState = STOP;
HeroShootLG::loader_state_t HeroShootLG::plateState = STOP;

HeroShootLG::AutoLoaderThread HeroShootLG::autoloader;
HeroShootLG::StuckDetectorThread HeroShootLG::stuckDetector;

void HeroShootLG::init(float loader_angle_per_bullet_, float plate_angle_per_bullet_, tprio_t stuck_detector_thread_prio, tprio_t automatic_thread_prio) {

    // initialize the parameter.
    loader_angle_per_bullet = loader_angle_per_bullet_;
    plate_angle_per_bullet = plate_angle_per_bullet_;

    // clear the bullet angle
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::reset_plate_accumulated_angle();

    stuckDetector.start(stuck_detector_thread_prio);
    autoloader.start(automatic_thread_prio);

}

void HeroShootLG::shoot() {

    if(loaderState == STOP) {
        ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
        if(loaded_bullet[0] && loaded_bullet[1]){    // if the next bullet place is loaded

            loader_target_angle += loader_angle_per_bullet;
            ShootSKD::set_loader_target(loader_target_angle);
            loaderState = LOADING;

            // update the status
            loaded_bullet[0] = loaded_bullet[1];
            loaded_bullet[1] = loaded_bullet[2];
            loaded_bullet[2] = loaded_bullet[3];
            loaded_bullet[3] = false;

        } else if(loaded_bullet[0] && !loaded_bullet[1]) {    // if the next bullet place is empty

            loader_target_angle += ( 2 * loader_angle_per_bullet);
            ShootSKD::set_loader_target(loader_target_angle);
            loaderState = LOADING;

            // update the status
            loaded_bullet[0] = loaded_bullet[2];
            loaded_bullet[1] = loaded_bullet[3];
            loaded_bullet[3] = false;

        } else if(!loaded_bullet[0]){
            loader_target_angle += ( 2 * loader_angle_per_bullet);
            ShootSKD::set_loader_target(loader_target_angle);
            loaderState = LOADING;

            // update the status
            loaded_bullet[0] = loaded_bullet[2];
            loaded_bullet[1] = loaded_bullet[3];
            loaded_bullet[3] = false;
            loaded_bullet[2] = (bool) palReadPad(GPIOE,GPIOE_PIN5);
        }
    }
}

void HeroShootLG::set_friction_wheels(float duty_cycle) {
    ShootSKD::set_friction_wheels(duty_cycle);
}

float HeroShootLG::get_friction_wheels_duty_cycle() {
    return ShootSKD::get_friction_wheels_duty_cycle();
}

void HeroShootLG::ForceStop() {
    ShootSKD::set_mode(ShootSKD::FORCED_RELAX_MODE);
}

HeroShootLG::loader_state_t HeroShootLG::get_loader_status() {
    if(loader_target_angle - ShootSKD::get_loader_accumulated_angle() > 3.0f && loaderState != STUCK) {
        loaderState = LOADING;
    } else if (loader_target_angle - ShootSKD::get_loader_accumulated_angle() < 3.0f && loaderState != STUCK){
        loaderState = STOP;
    }
    return loaderState;
}

HeroShootLG::loader_state_t HeroShootLG::get_plate_status() {
    if(plate_target_angle - ShootSKD::get_plate_accumulated_angle() > 2.0f && plateState != STUCK) {
        plateState = LOADING;
    } else if (plate_target_angle - ShootSKD::get_plate_accumulated_angle() < 2.0f && plateState!= STUCK) {
        plateState = STOP;
    }
    return plateState;
}

void HeroShootLG::StuckDetectorThread::main() {
    setName("Stuck_Detector");
    while(!shouldTerminate()) {
        if(get_loader_status() == LOADING &&
                    ShootSKD::get_loader_target_current() > LOADER_STUCK_THRESHOLD_CURRENT &&
                    ShootSKD::get_loader_actual_velocity() < LOADER_STUCK_THRESHOLD_VELOCITY){
            loaderState = STUCK;
            ShootSKD::set_loader_target(ShootSKD::get_loader_accumulated_angle() - 10.0f);  // Back up to ample space
        }
//        if(get_plate_status() == LOADING &&
//           ShootSKD::get_plate_target_current() > PLATE_STUCK_THRESHOLD_CURRENT &&
//           ShootSKD::get_plate_actual_velocity() < PLATE_STUCK_THRESHOLD_VELOCITY &&
//           load_bullet_count != 0) {
//            plateState = STUCK;
//            ShootSKD::set_plate_target(ShootSKD::get_plate_accumulated_angle() - 10.0f);  // Back up to ample space
//        }
        if(loaderState == STUCK || plateState == STUCK){
            sleep(TIME_MS2I(STUCK_REVERSE_TIME));
            loaderState = LOADING;
            ShootSKD::set_loader_target(loader_target_angle);
            ShootSKD::set_plate_target(plate_target_angle);
            plateState = LOADING;
        }
        sleep(TIME_MS2I(STUCK_DETECTOR_THREAD_INTERVAL));
    }
}

void HeroShootLG::AutoLoaderThread::main() {
    setName("Automation");

    while(!shouldTerminate()) {

        // Update sequence by detecting the last bullet place
        loaded_bullet[2] = (bool) palReadPad(GPIOE, GPIOE_PIN4);

        // When loaded_bullet[2] is true and an extra loading bullet is detected
        if(loaded_bullet[2] && (bool) palReadPad(GPIOE, GPIOE_PIN5)) {
            loaded_bullet[3] = true;
        }

        // loader load automatically.
        if(!loaded_bullet[0]  && loaded_bullet[2] && get_loader_status() == STOP) {
            ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
            // get rid of empty bullet place
            loaderState = LOADING;
            loader_target_angle += loader_angle_per_bullet;
            ShootSKD::set_loader_target(loader_target_angle);
            loaded_bullet[0] = loaded_bullet[1];
            loaded_bullet[1] = loaded_bullet[2];
            loaded_bullet[2] = loaded_bullet[3];
            loaded_bullet[3] = false;
            // Maybe no need to update the loaded_bullet[2] as the loaded_bullet[2] will refresh real timely.
        }

        // Plate load automatically.
        if( get_plate_status() == STOP && ((loaded_bullet[2] && !loaded_bullet[3] && loaded_bullet[0] && !loaded_bullet[1]) || (!loaded_bullet[2])) ) {
            ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
                // if top bullet place is (about to) be empty due to auto loading
            plateState = LOADING;
            plate_target_angle += plate_angle_per_bullet;
            ShootSKD::set_plate_target(plate_target_angle);
            // No need to update the sequence. The special situation has already been considered.
        }

        sleep(TIME_MS2I(AUTOLOADER_THREAD_INTERVAL));
    }

}