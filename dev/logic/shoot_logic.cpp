//
// Created by liuzikai on 2019-06-26.
//

/**
 * @file    shoot_logic.cpp
 * @brief   Logic-level module to control shooter. Support number-controlling shooting.
 *
 * @addtogroup shoot
 * @{
 */


#include "shoot_logic.h"

#include "shell.h"
#include "shoot_scheduler.h"
#include "referee_interface.h"
#include "buzzer_scheduler.h"
#include "vision.h"
#include <cmath>

ShootLG::mode_t ShootLG::mode = MANUAL_MODE;
float ShootLG::angle_per_bullet = 0;
int ShootLG::bullet_count = 0;
float ShootLG::shoot_target_number = 0;
ShootLG::shooter_state_t ShootLG::shooter_state = STOP;
ShootLG::StuckNHeatDetectorThread ShootLG::stuck_detector_thread;
chibios_rt::ThreadReference ShootLG::stuck_detector_ref;
ShootLG::BulletCounterThread ShootLG::bullet_counter_thread;
ShootLG::VisionShootThread ShootLG::vision_shoot_thread;

void ShootLG::init(float angle_per_bullet_, tprio_t stuck_detector_thread_prio, tprio_t bullet_counter_thread_prio,
                   tprio_t vision_shooting_thread_prio) {
    angle_per_bullet = angle_per_bullet_;
    stuck_detector_ref = stuck_detector_thread.start(stuck_detector_thread_prio);
    bullet_counter_thread.start(bullet_counter_thread_prio);
    vision_shoot_thread.start(vision_shooting_thread_prio);
}

void ShootLG::increment_bullet(int number_of_bullet) {
    bullet_count += number_of_bullet;
}

void ShootLG::set_bullet_count(int number_of_bullet) {
    bullet_count = number_of_bullet;
}

int ShootLG::get_bullet_count() {
    return bullet_count;
}

void ShootLG::set_friction_wheels(float duty_cycle) {
    ShootSKD::set_friction_wheels(duty_cycle);
}

float ShootLG::get_friction_wheels_duty_cycle() {
    return ShootSKD::get_friction_wheels_duty_cycle();
}

ShootLG::shooter_state_t ShootLG::get_shooter_state() {
    return shooter_state;
}

void ShootLG::shoot(float number_of_bullet, float number_per_second) {
    if (mode != MANUAL_MODE) {
        LOG_ERR("ShootLG: shoot() is called outside MANUAL_MODE");
        return;
    }

    shoot_target_number = number_of_bullet;

    shooter_state = SHOOTING;
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
    ShootSKD::set_loader_target_velocity(number_per_second * angle_per_bullet);
    ShootSKD::set_loader_target_angle(shoot_target_number * angle_per_bullet);
    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    {
        if (!stuck_detector_thread.started) {
            stuck_detector_thread.started = true;
            stuck_detector_thread.paused_once = false;
            chSchWakeupS(stuck_detector_ref.getInner(), 0);
        }
    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
}

void ShootLG::stop() {
    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    {
        bullet_count -= (int) roundf((float) ShootSKD::get_loader_accumulated_angle() / angle_per_bullet);
    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::set_loader_target_angle(0.0f);
    shooter_state = STOP;
}

void ShootLG::force_stop() {
    ShootSKD::set_mode(ShootSKD::FORCED_RELAX_MODE);
    stop();
}

void ShootLG::StuckNHeatDetectorThread::main() {
    setName("ShootLG_Stuck");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            if (shooter_state != SHOOTING) {
                started = false;
                chSchGoSleepS(CH_STATE_SUSPENDED);
            }
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        if (!paused_once) {
            sleep(TIME_MS2I(STUCK_DETECTOR_INITIAL_WAIT_INTERVAL));
            paused_once = true;
        }
        if (shooter_state != SHOOTING) {  // if shooter is stopped during the pausing time, terminate the process.
            continue;
        }

        if (ShootSKD::get_loader_target_current() > STUCK_THRESHOLD_CURRENT &&
            ShootSKD::get_loader_actual_velocity() < STUCK_THRESHOLD_VELOCITY) {
            stuck_count++;
        } else {
            stuck_count = 0;
        }

        if (stuck_count > STUCK_THRESHOLD_COUNT) {
            shooter_state = STUCK;
            LOG_WARN("Bullet loader stuck");
            ShootSKD::set_loader_target_angle(ShootSKD::get_loader_accumulated_angle() - STUCK_REVERSE_ANGLE);
            sleep(TIME_MS2I(STUCK_REVERSE_TIME));
            ShootSKD::set_loader_target_angle(shoot_target_number * angle_per_bullet);  // recover original target
            shooter_state = SHOOTING;
        }

        sleep(TIME_MS2I(STUCK_DETECTOR_THREAD_INTERVAL));
    }
}

void ShootLG::BulletCounterThread::main() {
    setName("ShootLG_Count");

    chEvtRegisterMask(&Referee::data_received_event, &data_received_listener, DATA_RECEIVED_EVENTMASK);

    while (!shouldTerminate()) {

        chEvtWaitAny(DATA_RECEIVED_EVENTMASK);

        eventflags_t flags = chEvtGetAndClearFlags(&data_received_listener);

        // Toggle Referee LED if any data is received
        LED::led_toggle(DEV_BOARD_LED_REFEREE);

        // Add bullets
        if (flags == Referee::SUPPLY_PROJECTILE_ACTION_CMD_ID &&
            Referee::supply_projectile_action.supply_robot_id == Referee::get_self_id() &&
            Referee::supply_projectile_action.supply_projectile_step == 2  // bullet fall
                ) {

            chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
            {
                bullet_count += (int) (Referee::supply_projectile_action.supply_projectile_num);
            }
            chSysUnlock();  /// --- EXIT S-Locked state ---
        }
    }
}

void ShootLG::VisionShootThread::main() {
    setName("ShootLG_Vision");

    chEvtRegisterMask(&Vision::shoot_time_updated_event, &vision_listener, VISION_UPDATED_EVENT_MASK);

    while (!shouldTerminate()) {

        chEvtWaitAny(VISION_UPDATED_EVENT_MASK);

        if (mode == VISION_AUTO_MODE) {

            time_msecs_t expected_shoot_time, command_issue_time;
            chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
            {
                expected_shoot_time = Vision::get_expected_shoot_time();
                command_issue_time = Vision::get_last_update_time();
            }
            chSysUnlock();  /// --- EXIT S-Locked state ---
            int64_t time_delta = (int64_t) expected_shoot_time - (int64_t) (SYSTIME);

            if (time_delta > 0) {
                sleep(TIME_MS2I(time_delta));  // wait for the remaining time
            }

            float target_angle = SHOOT_BULLET_COUNT * angle_per_bullet;

            // Shoot
            ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
            ShootSKD::reset_loader_accumulated_angle();
            ShootSKD::set_loader_target_velocity(SHOOT_BULLET_SPEED * angle_per_bullet);
            ShootSKD::set_loader_target_angle(target_angle);

            // Wait for ShootSKD to achieve the target
            while (mode == VISION_AUTO_MODE && ShootSKD::get_mode() == ShootSKD::LIMITED_SHOOTING_MODE &&
                   ShootSKD::get_loader_target_angle() == target_angle &&
                   target_angle - ShootSKD::get_loader_accumulated_angle() < 0.5 * angle_per_bullet) {

                sleep(TIME_MS2I(5));
            }

            // Update measured shoot delay
            if (mode == VISION_AUTO_MODE && ShootSKD::get_mode() == ShootSKD::LIMITED_SHOOTING_MODE &&
                ShootSKD::get_loader_target_angle() == target_angle) {  // make sure there is no interference

                Vision::update_measured_shoot_delay(SYSTIME - command_issue_time);
            }

            // Wait for some time
            sleep(TIME_MS2I(WAIT_TIME_BETWEEN_SHOOTS));

        }  // otherwise, discard the event
    }
}

/** @} */
