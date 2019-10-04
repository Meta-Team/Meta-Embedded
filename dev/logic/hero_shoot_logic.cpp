//
// Created by 钱晨 on 2019-07-03.
//

#include "hero_shoot_logic.h"

#include "shell.h"
#include "referee_interface.h"
#include "shoot_scheduler.h"

float HeroShootLG::loader_angle_per_bullet = 0.0f;
float HeroShootLG::plate_angle_per_bullet = 0.0f;
float HeroShootLG::loader_target_angle = 0.0f;
float HeroShootLG::plate_target_angle = 0.0f;

int HeroShootLG::bullet_in_tube = 0;
bool HeroShootLG::should_shoot = false;

HeroShootLG::motor_state_t HeroShootLG::loader_state = STOP;
HeroShootLG::motor_state_t HeroShootLG::plate_state = STOP;

HeroShootLG::LoaderCalibrateThread HeroShootLG::loaderCalibrateThread;
HeroShootLG::LoaderStuckDetectorThread HeroShootLG::loaderStuckDetector;
HeroShootLG::PlateStuckDetectorThread HeroShootLG::plateStuckDetector;
HeroShootLG::LoaderThread HeroShootLG::loaderThread;
HeroShootLG::PlateThread HeroShootLG::plateThread;


void HeroShootLG::init(float loader_angle_per_bullet_, float plate_angle_per_bullet_, tprio_t loader_calibrate_prio,
                       tprio_t loader_thread_prio, tprio_t plate_thread_prio,
                       tprio_t loader_stuck_detector_prio, tprio_t plate_stuck_detector_prio) {

    // Initialize parameters
    loader_angle_per_bullet = loader_angle_per_bullet_;
    plate_angle_per_bullet = plate_angle_per_bullet_;

    // Clear the bullet angle
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::reset_plate_accumulated_angle();

    ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);

    loaderCalibrateThread.start(loader_calibrate_prio);
    chThdSleepMilliseconds(3000);// wait loader to calibrate
    loaderThread.start(loader_thread_prio);
    plateThread.start(plate_thread_prio);
    loaderStuckDetector.start(loader_stuck_detector_prio);
    plateStuckDetector.start(plate_stuck_detector_prio);

}

bool HeroShootLG::get_loader_exit_status() {
    return !(bool) palReadPad(GPIOF, GPIOF_PIN0);
}

bool HeroShootLG::get_plate_exit_status() {
    return !(bool) palReadPad(GPIOF, GPIOF_PIN1);
}

bool HeroShootLG::get_loading_status() {
    return (loader_state == STOP && get_loader_exit_status());
}

void HeroShootLG::shoot() {
    if (ShootSKD::get_mode() != ShootSKD::LIMITED_SHOOTING_MODE) ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
    if (loader_state == STOP && get_loader_exit_status()) {
        should_shoot = true;
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

float HeroShootLG::measure_loader_exit_status() {
    int cnt = 0;
    for (int i = 0; i < 5; i++) {
        cnt += get_loader_exit_status();
        chThdSleepMicroseconds(200    );
    }
    return (float) cnt / 5.0f;
}
void HeroShootLG::LoaderCalibrateThread::main() {
    setName("LoaderCalibrate");
    bool calibrate_success = false;
    bool loader_exit_status_sequence[2] = {false, false};
    ShootSKD::load_pid_params(CALIBRATE_PID_BULLET_LOADER_A2V_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS,SHOOT_PID_BULLET_PLATE_A2V_PARAMS,SHOOT_PID_BULLET_PLATE_V2I_PARAMS);
    while(!shouldTerminate()) {
        loader_exit_status_sequence[0] = loader_exit_status_sequence[1];
        loader_exit_status_sequence[1] = get_loader_exit_status();
        if(!calibrate_success && loader_target_angle == 0.0f ){
            ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
            loader_target_angle = 666.6f;
            ShootSKD::set_loader_target_angle(loader_target_angle);
        }
        if((loader_exit_status_sequence[0] && !loader_exit_status_sequence[1]) && loader_target_angle == 666.6f){
            loader_target_angle = ShootSKD::get_loader_accumulated_angle();
            ShootSKD::set_loader_target_angle(loader_target_angle);
            ShootSKD::load_pid_params(SHOOT_PID_BULLET_LOADER_A2V_PARAMS,SHOOT_PID_BULLET_LOADER_V2I_PARAMS,SHOOT_PID_BULLET_PLATE_A2V_PARAMS,SHOOT_PID_BULLET_PLATE_V2I_PARAMS);
            loader_target_angle += 40.0f;
            ShootSKD::set_loader_target_angle(loader_target_angle);
        }
        if (loader_target_angle != 666.6f && ABS_IN_RANGE(loader_target_angle - ShootSKD::get_loader_accumulated_angle(),3.0f)){
            ShootSKD::reset_loader_accumulated_angle();
            ShootSKD::set_loader_target_angle(0);
            loader_target_angle = 0.0f;
            calibrate_success = true;

            chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
            chSchGoSleepS(CH_STATE_SUSPENDED);
            chSysUnlock();  /// --- EXIT S-Locked state ---
        }
        sleep(CALIBRATE_THREAD_INTERVAL);
    }
}

void HeroShootLG::LoaderStuckDetectorThread::main() {
    setName("StuckDetector1");

    int stuck_pend_time = 0;

    while (!shouldTerminate()) {

        if (loader_state == LOADING) {
            if (ShootSKD::get_loader_target_current() > LOADER_STUCK_THRESHOLD_CURRENT &&
                ShootSKD::get_loader_actual_velocity() < LOADER_STUCK_THRESHOLD_VELOCITY) {

                stuck_pend_time++;  // Back up to ample space

            } else {

                stuck_pend_time = 0;
            }
        }

        if (stuck_pend_time > 50) {
            loader_state = STUCK;
            ShootSKD::set_loader_target_angle(ShootSKD::get_loader_accumulated_angle() - STUCK_REVERSE_ANGLE);

            sleep(TIME_MS2I(STUCK_REVERSE_TIME));

            loader_state = LOADING;
            ShootSKD::set_loader_target_angle(loader_target_angle);

            stuck_pend_time = 0;
        }

        sleep(TIME_MS2I(STUCK_DETECTOR_THREAD_INTERVAL));
    }
}

void HeroShootLG::PlateStuckDetectorThread::main() {

    setName("StuckDetector2");

    int stuck_pend_time = 0;

    while (!shouldTerminate()) {

        if (plate_state == LOADING) {
            if (ShootSKD::get_plate_target_current() > PLATE_STUCK_THRESHOLD_CURRENT &&
                ShootSKD::get_plate_actual_velocity() < PLATE_STUCK_THRESHOLD_VELOCITY) {

                stuck_pend_time++;  // Back up to ample space

            } else {

                stuck_pend_time = 0;
            }
        }

        if (stuck_pend_time > 60) {
            plate_state = STUCK;
            ShootSKD::set_plate_target_angle(ShootSKD::get_plate_accumulated_angle() - STUCK_REVERSE_ANGLE);

            sleep(TIME_MS2I(STUCK_REVERSE_TIME));

            plate_state = LOADING;
            ShootSKD::set_plate_target_angle(plate_target_angle);

            stuck_pend_time = 0;
        }

        sleep(TIME_MS2I(STUCK_DETECTOR_THREAD_INTERVAL));
    }
}

void HeroShootLG::LoaderThread::main() {

    setName("HeroShootLoader");

    while (!shouldTerminate()) {

        if (loader_state != STUCK) {

            if (ABS_IN_RANGE(loader_target_angle - ShootSKD::get_loader_accumulated_angle(),
                             LOADER_REACH_TARGET_RANGE)) {
                loader_state = STOP;
            } else {
                loader_state = LOADING;
            }

            if (loader_state == STOP) {
                unsigned cnt1 = 0;
                while (!should_shoot && cnt1 < 600) {
                    sleep(TIME_MS2I(1));
                    cnt1++;
                }
                if (should_shoot || !get_loader_exit_status()) {
                    unsigned cnt2 = 0;
                    if (should_shoot) {
                        should_shoot = false;
                        if(bullet_in_tube > 0) bullet_in_tube--;
                    }
                    loader_target_angle += loader_angle_per_bullet;
                    loader_state = LOADING;
                    ShootSKD::set_loader_target_angle(loader_target_angle);
                    while (cnt2 < 400) {
                        sleep(TIME_MS2I(1));
                        cnt2++;
                    }
                }
            }

        }

        sleep(TIME_MS2I(LOADER_THREAD_INTERVAL));
    }

}

void HeroShootLG::PlateThread::main() {

    setName("HeroShootPlate");

    int last_load_time = SYSTIME;

    while (!shouldTerminate()) {

        if (plate_state != STUCK) {

            if (!last_plate_exit_status && get_plate_exit_status()) {
                bullet_in_tube++;
            }
            last_plate_exit_status = get_plate_exit_status();

            if (ABS_IN_RANGE(plate_target_angle - ShootSKD::get_plate_accumulated_angle(), PLATE_REACH_TARGET_RANGE)) {
                plate_state = STOP;
            } else {
                plate_state = LOADING;
            }

            if (plate_state == STOP) {
//                sleep(TIME_MS2I(200));
                if (bullet_in_tube < 3 && SYSTIME - last_load_time > 500) {
                    plate_target_angle += plate_angle_per_bullet;
                    ShootSKD::set_plate_target_angle(plate_target_angle);
                    last_load_time = SYSTIME;
                }
            }

        }

        sleep(TIME_MS2I(PLATE_THREAD_INTERVAL));
    }

}
