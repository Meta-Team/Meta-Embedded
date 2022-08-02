//
// Created by Quoke on 11/24/2021.
//

#include "Communicator.h"
#include "buzzer_scheduler.h"

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
    CANMotorIF::motor_feedback[0].reset_accumulate_angle();
    CANMotorIF::motor_feedback[1].reset_accumulate_angle();
    uint8_t prev_rxmode = 0x00;
    time_msecs_t grab_start_time = 0;
    while(!shouldTerminate()) {
        tx_angles[1] = (uint8_t)(((int16_t)(CANMotorIF::motor_feedback[0].accumulate_angle() / 360.0f * 8192.0f)) >> 8);
        tx_angles[2] = (uint8_t)((int16_t)(CANMotorIF::motor_feedback[0].accumulate_angle() / 360.0f * 8192.0f));
        tx_angles[3] = (uint8_t)(((int16_t)(CANMotorIF::motor_feedback[1].accumulate_angle() / 360.0f * 8192.0f)) >> 8);
        tx_angles[4] = (uint8_t)((int16_t)(CANMotorIF::motor_feedback[1].accumulate_angle() / 360.0f * 8192.0f));
        send_data(tx_angles, 5);
        for (unsigned char i : buffer) {
            Shell::printf(" %d,", i);
        }
        Shell::printf(SHELL_NEWLINE_STR);
        if(rxmode == 0x01) {
            HapticLG::haptic_device_mode = HapticLG::torqueMode;
            int16_t target0 = target_torque[0];
            int16_t target1 = target_torque[1];
            VAL_CROP(target0, 16000, -16000);
            VAL_CROP(target1, 16000, -16000);
            HapticLG::target_current[0] = -target1;
            HapticLG::target_current[1] = (int16_t) ((float)target0/2);
            HapticLG::target_current[2] = (int16_t) ((float)-target0/2);
            LED::led_on(1);
            if(prev_rxmode == 0x00) {
                palWritePad(GPIOE, GPIOE_PIN4, 1);
                grab_start_time = SYSTIME;
            }
            if(SYSTIME - grab_start_time > 500) {
                palWritePad(GPIOE, GPIOE_PIN4, 0);
            }

        } else if (rxmode == 0x00) {
            HapticLG::haptic_device_mode = HapticLG::torqueMode;
            HapticLG::target_current[0] = HapticLG::target_current[1] = HapticLG::target_current[2] = 0;
            palWritePad(GPIOE, GPIOE_PIN4, 0);
            LED::led_off(1);
        }
        prev_rxmode = rxmode;
        sleep(TIME_MS2I(25));
    }
}

void Communicator::RxThread::main() {
    setName("USBRxThread");
    while(!shouldTerminate()) {
        chnReadTimeout(&USBSerialIF::SDU, buffer, pak_size, TIME_INFINITE);
        target_torque[0] = (int16_t)(buffer[1] << 8 | buffer[0]);
        target_torque[1] = (int16_t)(buffer[3] << 8 | buffer[2]);
        rxmode = buffer[4];
        last_update_time = SYSTIME;
    }
}