//
// Created by 钱晨 on 2019-07-03.
//

#include "hero_shoot_logic.h"

void HeroShootLG::init(float loader_angle_per_bullet_, float plate_angle_per_bullet_, tprio_t stuck_detector_thread_prio, tprio_t automatic_thread_prio) {

    // initialize the parameter.
    loader_angle_per_bullet = loader_angle_per_bullet_;
    plate_angle_per_bullet = plate_angle_per_bullet_;

    // reset the target angle.
    loader_target_angle = 0.0f;
    plate_target_angle = 0.0f;

    // clear the bullet angle
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::reset_plate_accumulated_angle();

    // clear the bullet
    loaded_bullet[0] = loaded_bullet[1] = loaded_bullet[2] = loaded_bullet[3] = false;

    stuckDetector.start(stuck_detector_thread_prio);
    automation.start(automatic_thread_prio);

}
void HeroShootLG::shoot() {
    if(loaderState == STOP) {
        if(loaded_bullet[0] && loaded_bullet[1]){

            loader_target_angle += loader_angle_per_bullet;
            ShootSKD::set_loader_target(loader_target_angle);
            loaderState = LOADING;

            // update the status
            loaded_bullet[0] == loaded_bullet[1];
            loaded_bullet[1] == loaded_bullet[2];
            loaded_bullet[2] == loaded_bullet[3];
            loaded_bullet[3] == false;

        } else if(loaded_bullet[0] && !loaded_bullet[1]) {

            loader_target_angle += ( 2 * loader_angle_per_bullet);
            ShootSKD::set_loader_target(loader_target_angle);
            loaderState = LOADING;

            // update the status
            loaded_bullet[0] == loaded_bullet[2];
            loaded_bullet[1] == loaded_bullet[3];
            loaded_bullet[3] == false;

        }
    }
}

void HeroShootLG::ForceStop() {
    ShootSKD::set_mode(ShootSKD::FORCED_RELAX_MODE);

    load_bullet_count = 0;
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::reset_plate_accumulated_angle();
    loader_target_angle = plate_target_angle = 0;
    plateState = FORCESTOP;
}
void HeroShootLG::StuckDetectorThread::main() {
    setName("Stuck_Detector");
    while(!shouldTerminate()) {
        if(loaderState == LOADING &&
           ShootSKD::get_loader_target_current() > LOADER_STUCK_THRESHOLD_CURRENT &&
           ShootSKD::get_loader_actual_velocity() < LOADER_STUCK_THRESHOLD_VELOCITY){
            loaderState = STUCK;
            ShootSKD::set_loader_target(ShootSKD::get_loader_accumulated_angle() - 10.0f);
        }
        if(plateState == LOADING &&
           ShootSKD::get_plate_target_current() > PLATE_STUCK_THRESHOLD_CURRENT &&
           ShootSKD::get_plate_actual_velocity() < PLATE_STUCK_THRESHOLD_VELOCITY &&
           load_bullet_count != 0) {
            plateState = STUCK;
            ShootSKD::set_plate_target(ShootSKD::get_plate_accumulated_angle() - 10.0f);
        }
        if(loaderState == STUCK || plateState == STUCK){
            sleep(TIME_MS2I(STUCK_REVERSE_TIME));
            loaderState == LOADING;
            plateState == LOADING;
        }
        sleep(TIME_MS2I(STUCK_DETECTOR_THREAD_INTERVAL));
    }
}
// Used for handle the plate motion as well as load logic.
void HeroShootLG::AutomateThread::main() {
    setName("Automation");

    while(!shouldTerminate()) {

        // Update the last place loaded bullet Sequence
        loaded_bullet[2] = (bool) palReadPad(GPIOE, GPIOE_PIN4);

        // When loaded_bullet[2] is true and the another
        if(loaded_bullet[2]== true && (bool) palReadPad(GPIOE, GPIOE_PIN5)) {
            loaded_bullet[3] == true;
        }

        // Update the loader status (LOADING OR STOP)
        if(loader_target_angle - ShootSKD::get_loader_accumulated_angle() > 2.0f && loaderState != STUCK) {
            loaderState = LOADING;
        } else if (loader_target_angle - ShootSKD::get_loader_accumulated_angle() < 2.0f && loaderState != STUCK){
            loaderState = STOP;
        }

        // Update the plate status (LOADING OR STOP)
        if(plate_target_angle - ShootSKD::get_plate_accumulated_angle() > 2.0f && plateState != STUCK) {
            plateState = LOADING;
        } else if (plate_target_angle - ShootSKD::get_plate_accumulated_angle() < 2.0f && plateState!= STUCK) {
            plateState = STOP;
        }

        // loader load automatically.
        if(loaded_bullet[0] == false && loaded_bullet[2] == true && loaderState == STOP) {
            loaderState == LOADING;
            loader_target_angle += loader_angle_per_bullet;
            ShootSKD::set_loader_target(loader_target_angle);
            loaded_bullet[0] = loaded_bullet[1];
            loaded_bullet[1] = loaded_bullet[2];
            if(loaded_bullet[3] == true){
                loaded_bullet[2] = loaded_bullet[3];
                loaded_bullet[3] = false;
            }
            // Maybe no need to update the loaded_bullet[2] as the loaded_bullet[2] will refresh real timely.
        }

        // Plate load automatically.
        if( plateState == STOP && (loaded_bullet[2] == false && loaded_bullet[3] == false) &&
            (loaded_bullet[0] == true && loaded_bullet[1] == false && loaded_bullet[2] == true && loaded_bullet[3] == false)) {
            plateState == LOADING;
            plate_target_angle += plate_angle_per_bullet;
            ShootSKD::set_plate_target(plate_target_angle);
            // No need to update the sequence. The special situation has already been considered.
        }

        sleep(TIME_MS2I(AUTOMATION_THREAD_INTERVAL));
    }

}