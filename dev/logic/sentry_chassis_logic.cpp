//
// Created by liuzikai on 2019-07-15.
//

#include "sentry_chassis_logic.h"

#include "sentry_chassis_scheduler.h"

SChassisLG::mode_t SChassisLG::mode = FORCED_RELAX_MODE;
float SChassisLG::manual_dest = 0;
float SChassisLG::radius = 50.0f;
bool SChassisLG::random_mode = false;
time_msecs_t SChassisLG::last_attack_time = 0;
constexpr float SChassisLG::terminals[6];
int SChassisLG::prev_terminal = 0;
int SChassisLG::next_terminal = 5;

SChassisLG::DirectionSwitchThread SChassisLG::directionSwitchThread;
chibios_rt::ThreadReference SChassisLG::directionThreadReference;

void SChassisLG::init(tprio_t direction_switch_thd_prio) {
    directionSwitchThread.started = true;
    directionThreadReference = directionSwitchThread.start(direction_switch_thd_prio);
}

void SChassisLG::set_mode(SChassisLG::mode_t value) {
    if (value == mode) return;

    mode = value;

    /// NOTICE: reset origin every time switching mode
    SChassisSKD::reset_origin();
    manual_dest = 0;

    if (mode == FORCED_RELAX_MODE) {
        SChassisSKD::set_mode(SChassisSKD::FORCED_RELAX_MODE);
    } else {
        SChassisSKD::set_mode(SChassisSKD::ABS_DEST_MODE);

        if (mode == MANUAL_MODE) {

            SChassisSKD::set_destination(manual_dest);

        } else if (mode == SHUTTLE_MODE || mode == FINAL_AUTO_MODE) {

            // Resume the thread
            chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
            if (!directionSwitchThread.started) {
                directionSwitchThread.started = true;
                chSchWakeupS(directionThreadReference.getInner(), 0);
            }
            chSysUnlock();  /// --- EXIT S-Locked state ---

            if (mode == SHUTTLE_MODE) {
                SChassisSKD::set_destination(radius);
            } else {
                prev_terminal = 0;
                next_terminal = 5;
                SChassisSKD::set_destination(terminals[next_terminal]);
            }
        }
    }
}

SChassisLG::mode_t SChassisLG::get_mode() {
    return mode;
}

void SChassisLG::set_manual_dest(float dest) {
    manual_dest = dest;
    SChassisSKD::set_destination(manual_dest);
}

float SChassisLG::get_manual_dest() {
    return manual_dest;
}

void SChassisLG::set_shuttle_radius(float shuttle_radius) {
    radius = shuttle_radius;
}

float SChassisLG::get_shuttle_radius() {
    return radius;
}

bool SChassisLG::get_escaping_status() {
    return random_mode;
}

void SChassisLG::start_escaping() {
    last_attack_time = SYSTIME;
    if (!random_mode) {
        random_mode = true;
        update_next_terminal();
    }
}

void SChassisLG::update_next_terminal() {

    if (random_mode) {

        int dest = SYSTIME % 6;  // get a random index between 0 and 5 and decide the next terminal accordingly

        if (dest == next_terminal) {
            next_terminal = prev_terminal;
            prev_terminal = dest;
        } else {
            if ((dest - next_terminal) * SChassisIF::present_velocity() > 0) dest = 5 - dest;
            prev_terminal = next_terminal;
            next_terminal = dest;
        }
    } else {

        if (next_terminal == 0) {
            prev_terminal = next_terminal;
            next_terminal = 5;
        } else {
            prev_terminal = next_terminal;
            next_terminal = 0;
        }
    }
    SChassisSKD::set_destination(terminals[next_terminal]);
}

void SChassisLG::stop_escaping() {
    random_mode = false;

    // Return to cruising mode
    if (SChassisSKD::present_velocity() > 0)
        next_terminal = 5;
    else
        next_terminal = 0;

    SChassisSKD::set_destination(terminals[next_terminal]);
}

void SChassisLG::DirectionSwitchThread::main() {
    setName("Chassis_Switch");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        if (mode != SHUTTLE_MODE && mode != FINAL_AUTO_MODE) {
            started = false;
            chSchGoSleepS(CH_STATE_SUSPENDED);
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        float present_position = SChassisSKD::present_position();

        if (mode == SHUTTLE_MODE) {

            if (present_position > radius - DEST_TOLERANT_RANGE) {
                SChassisSKD::set_destination(-radius);
            } else if (present_position < -radius + DEST_TOLERANT_RANGE) {
                SChassisSKD::set_destination(radius);
            }
        } else if (mode == FINAL_AUTO_MODE) {
            
            if (random_mode && not WITHIN_RECENT_TIME(last_attack_time, STOP_ESCAPING_AFTER)) stop_escaping();

            if (ABS_IN_RANGE(present_position - terminals[next_terminal], DEST_TOLERANT_RANGE)) {
                update_next_terminal();
            }
        }

        sleep(TIME_MS2I(DIRECTION_INSPECTION_INTERVAL));

    }
}