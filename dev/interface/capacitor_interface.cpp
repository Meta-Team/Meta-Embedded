//
// Created by liuzikai on 2019-07-14.
//

#include "capacitor_interface.h"

// CAN-relate attributes
CANInterface *CapacitorIF::can_ = nullptr;
CANTxFrame CapacitorIF::txFrame;

// Feedback-relate attributes
time_msecs_t CapacitorIF::last_feedback_time = 0;
float CapacitorIF::input_voltage;
float CapacitorIF::input_current;
float CapacitorIF::output_power;
float CapacitorIF::capacitor_voltage;

// Update thread
CapacitorIF::SuperCapacitorInitThread CapacitorIF::superCapacitorInitThread;

void CapacitorIF::init(CANInterface *can_interface, tprio_t INIT_THREAD_PRIO) {
    can_ = can_interface;
    can_->register_callback(0x211, 0x211, CapacitorIF::process_feedback);
    superCapacitorInitThread.start(INIT_THREAD_PRIO);
}

void CapacitorIF::set_power(float input_power) {
    chSysLock();
    {
        ///--- Enter S-Lock state ---///
        // Filling the super capacitor tx frame.
        txFrame.IDE = CAN_IDE_STD;
        txFrame.SID = 0x210;
        txFrame.RTR = CAN_RTR_DATA;
        txFrame.DLC = 0x02;  // 2 bytes of data
        txFrame.data8[0] = (uint8_t)(((uint16_t) (input_power * 100)) >> 8U);
        txFrame.data8[1] = (uint8_t)((uint16_t) (input_power * 100));
    }
    chSysUnlock();
    can_->send_msg(&txFrame);
}

void CapacitorIF::process_feedback(CANRxFrame const *rxmsg) {
    input_voltage =     (float)(rxmsg->data16[0]) / 100.0f;
    capacitor_voltage = (float)(rxmsg->data16[1]) / 100.0f;
    input_current =     (float)(rxmsg->data16[2])/100.0f;
    output_power =      (float)(rxmsg->data16[3])/100.0f;
    last_feedback_time = SYSTIME;
}

void CapacitorIF::SuperCapacitorInitThread::main() {
    setName("SuperInit");
    bool set_timeout;
    int start_time = SYSTIME;
    while(!shouldTerminate()) {
        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        if (output_power == 55.0f && !set_timeout) {
            chSchGoSleepS(CH_STATE_SUSPENDED);
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        if(SYSTIME - start_time > 3000) set_timeout = true;
        CapacitorIF::set_power(55.0f);
        sleep(TIME_MS2I(INIT_THREAD_INTERVAL));
    }
}