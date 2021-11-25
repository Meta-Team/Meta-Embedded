//
// Created by Quoke on 11/24/2021.
//

#include "Communicator.h"

Communicator::CommunicatorThd Communicator::communicator_thd;
uint8_t Communicator::tx_angles[CANMotorCFG::MOTOR_COUNT*2+1];

void Communicator::init(tprio_t communicator_prio_) {
    communicator_thd.start(communicator_prio_);
}

void Communicator::CommunicatorThd::main() {
    setName("Communicator");
    while(!shouldTerminate()) {
        tx_angles[1] = (uint8_t)(((int16_t)(CANMotorInterface::motor_feedback[0].accumulate_angle()/360.0f*8192.0f)) >> 8);
        tx_angles[2] = (uint8_t)((int16_t)(CANMotorInterface::motor_feedback[0].accumulate_angle()/360.0f*8192.0f));
        tx_angles[3] = (uint8_t)(((int16_t)(CANMotorInterface::motor_feedback[1].accumulate_angle()/360.0f*8192.0f)) >> 8);
        tx_angles[4] = (uint8_t)((int16_t)(CANMotorInterface::motor_feedback[1].accumulate_angle()/360.0f*8192.0f));
        VirtualCOMPort::send_data(tx_angles, 5);
        Shell::printf("rx torque: %d %d" SHELL_NEWLINE_STR, VirtualCOMPort::target_torque[0], VirtualCOMPort::target_torque[1]);
        Shell::printf("Buffer Content:" SHELL_NEWLINE_STR);
        for(int i = 0; i < 5; i++) {
            Shell::printf("%d ", VirtualCOMPort::rxbuffer[i]);
        }
        Shell::printf(SHELL_NEWLINE_STR);
        sleep(TIME_MS2I(20));
    }
}
