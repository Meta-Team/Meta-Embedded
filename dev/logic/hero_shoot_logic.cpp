//
// Created by 钱晨 on 2019-07-03.
//

#include "hero_shoot_logic.h"

void HeroShootLG::init(float loader_angle_per_bullet_, float plate_angle_per_bullet_, tprio_t stuck_detector_thread_prio, tprio_t automatic_thread_prio) {
    loader_angle_per_bullet = loader_angle_per_bullet_;
    plate_angle_per_bullet = plate_angle_per_bullet_;
    // clear the bullet.

    loaded_bullet[0] = loaded_bullet[1] = loaded_bullet[2] = loaded_bullet[3] = false;

    stuckDetector.start(stuck_detector_thread_prio);
    automation.start(automatic_thread_prio);
}
void HeroShootLG::shoot() {

}
void HeroShootLG::StuckDetectorThread::main() {
    setName("Stuck_Detector");
    while(!shouldTerminate()) {
        if(loaderState == LOADING &&
           ShootSKD::get_loader_target_current() > LOADER_STUCK_THRESHOLD_CURRENT &&
           ShootSKD::get_loader_actual_velocity() < LOADER_STUCK_THRESHOLD_VELOCITY){
            loaderState = STUCK;

        }
        if(plateState == LOADING &&
           ShootSKD::get_plate_target_current() > PLATE_STUCK_THRESHOLD_CURRENT &&
           ShootSKD::get_plate_actual_velocity() < PLATE_STUCK_THRESHOLD_VELOCITY &&
           load_bullet_count == 0) {
            plateState = STUCK;
        }
    }
}
// Used for handle the plate motion as well as load logic.
void HeroShootLG::AutomateThread::main() {
    setName("Automation");

    while(!shouldTerminate()) {
        // update the actual angle.

    }

}