//
// Created by liuzikai on 2019-07-14.
//

#include "super_capacitor_port.h"

CANInterface *SuperCapacitor::can_ = nullptr;
CANInterface::cap_feedback_t *SuperCapacitor::feedback;
CANTxFrame SuperCapacitor::txFrame;
time_msecs_t SuperCapacitor::last_feedback_time = 0;
SuperCapacitor::SuperCapacitorInitThread SuperCapacitor::superCapacitorInitThread;

void SuperCapacitor::init(CANInterface *can_interface, tprio_t INIT_THREAD_PRIO) {
    can_ = can_interface;
    feedback = can_->get_cap_feedback_address();
    superCapacitorInitThread.start(INIT_THREAD_PRIO);
}

void SuperCapacitor::set_power(float input_power) {
    txFrame.IDE = CAN_IDE_STD;
    txFrame.SID = 0x210;
    txFrame.RTR = CAN_RTR_DATA;
    txFrame.DLC = 0x02;  // 2 bytes of data
    txFrame.data8[0] = (uint8_t)(((uint16_t) (input_power * 100)) >> 8U);
    txFrame.data8[1] = (uint8_t)((uint16_t) (input_power * 100));
    can_->send_cap_msg(&txFrame);
}

void SuperCapacitor::SuperCapacitorInitThread::main() {
    setName("SuperInit");
    bool set_timeout;
    int start_time = SYSTIME;
    while(!shouldTerminate()) {
        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        if (feedback->output_power == 55.0f && !set_timeout) {
            chSchGoSleepS(CH_STATE_SUSPENDED);
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        if(SYSTIME - start_time > 3000) set_timeout = true;
        SuperCapacitor::set_power(55.0f);
        sleep(TIME_MS2I(INIT_THREAD_INTERVAL));
    }
}