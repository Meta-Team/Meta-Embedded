//
// Created by Quoke on 11/24/2021.
//

#include "Communicator.h"

Communicator::CommunicatorThd Communicator::communicator_thd;
Communicator::RxThread Communicator::rx_thd;
uint8_t Communicator::tx_angles[CANMotorCFG::MOTOR_COUNT*2+1];
int16_t Communicator::target_torque[CANMotorCFG::MOTOR_COUNT];
uint8_t Communicator::rxmode;
time_msecs_t Communicator::last_update_time;
uint8_t Communicator::buffer[Communicator::pak_size];


void Communicator::init(tprio_t communicator_prio_, tprio_t rx_prio_) {
    communicator_thd.start(communicator_prio_);
    rx_thd.start(rx_prio_);
}

void Communicator::send_data(uint8_t *data, unsigned int size) {
    chnWriteTimeout(&USBSerialIF::SDU, data,  size, TIME_INFINITE);
}

void Communicator::CommunicatorThd::main() {
    setName("Communicator");
    while(!shouldTerminate()) {
        tx_angles[1] = (uint8_t)(((int16_t)(CANMotorIF::motor_feedback[0].accumulate_angle() / 360.0f * 8192.0f)) >> 8);
        tx_angles[2] = (uint8_t)((int16_t)(CANMotorIF::motor_feedback[0].accumulate_angle() / 360.0f * 8192.0f));
        tx_angles[3] = (uint8_t)(((int16_t)(CANMotorIF::motor_feedback[1].accumulate_angle() / 360.0f * 8192.0f)) >> 8);
        tx_angles[4] = (uint8_t)((int16_t)(CANMotorIF::motor_feedback[1].accumulate_angle() / 360.0f * 8192.0f));
        send_data(tx_angles, 5);
        Shell::printf("torque:   %d %d, mode: %d" SHELL_NEWLINE_STR, target_torque[0], target_torque[1], rxmode);
        Shell::printf("rxbuffer:");
        for (int i = 0; i < pak_size; i++) {
            Shell::printf(" %d,", buffer[i]);
        }
        Shell::printf(SHELL_NEWLINE_STR);
        if(HapticLG::device_calibrated()&&rxmode == 0x01) {
            HapticLG::haptic_device_mode = HapticLG::torqueMode;
            int16_t target0 = target_torque[0];
            int16_t target1 = target_torque[1];
            VAL_CROP(target0, 16000, -16000);
            VAL_CROP(target1, 16000, -16000);
            HapticLG::target_current[0] = target0;
            HapticLG::target_current[1] = target1;
        } else if (HapticLG::device_calibrated()&&rxmode == 0x00) {
            HapticLG::haptic_device_mode = HapticLG::followMode;
            HapticLG::target_angle[0] = HapticLG::target_angle[1] = 0.0f;
        }
        sleep(TIME_MS2I(20));
    }
}

void Communicator::RxThread::main() {
    setName("USBRxThread");
    while(!shouldTerminate()) {
        chnReadTimeout(&USBSerialIF::SDU, buffer, 5, TIME_INFINITE);
        target_torque[0] = (int16_t)(buffer[1] << 8 | buffer[0]);
        target_torque[1] = (int16_t)(buffer[3] << 8 | buffer[2]);
        rxmode = buffer[4];
        last_update_time = SYSTIME;
    }
}
