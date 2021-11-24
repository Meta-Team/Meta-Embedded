//
// Created by Quoke on 11/24/2021.
//

#include "Communicator.h"

Communicator::CommunicatorSKD Communicator::communicator_skd;

void Communicator::init(tprio_t communicator_prio_) {
    communicator_skd.start(communicator_prio_);
}

void Communicator::CommunicatorSKD::main() {
    setName("Communicator");
    while(!shouldTerminate()) {
        auto angle1 = (uint32_t) CANMotorInterface::motor_feedback[0].accumulate_angle();
        auto angle2 = (uint32_t) CANMotorInterface::motor_feedback[1].accumulate_angle();
        uint8_t txdata[9] = {0xFF,
                             (uint8_t) (angle1 >> 24),
                             (uint8_t) (angle1 >> 16),
                             (uint8_t) (angle1 >> 8),
                             (uint8_t) (angle1),
                             (uint8_t) (angle2 >> 24),
                             (uint8_t) (angle2 >> 16),
                             (uint8_t) (angle2 >> 8),
                             (uint8_t) (angle2)};
        VirtualCOMPort::send_angles(txdata, 9);
        sleep(TIME_MS2I(20));
    }
}
