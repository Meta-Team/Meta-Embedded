//
// Created by Ye Anbang on 2019/2/2.
//
#include "robotic_arm.h"
#include "common_macro.h"

RoboticArm::clamp_status_t RoboticArm::_clamp_status = RoboticArm::CLAMP_RELAX;
float RoboticArm::motor_actual_angle;
static int motor_target_current;
CANInterface *RoboticArm::can = nullptr;

float RoboticArm::get_motor_actual_angle() {
    return motor_actual_angle;
}

RoboticArm::clamp_status_t RoboticArm::get_clamp_status() {
    return _clamp_status;
}

void RoboticArm::clamp_action(RoboticArm::clamp_status_t target_status) {
    _clamp_status = target_status;
    palWritePad(GPIOH, GPIOH_ROBOTIC_ARM_CLAMP, _clamp_status);
}

void RoboticArm::init(CANInterface *can_interface) {
    can = can_interface;
    can->register_callback(0x205, 0x205, process_motor_feedback);
}

void RoboticArm::process_motor_feedback(CANRxFrame const *rxmsg) {
    if (rxmsg->SID != 0x205) return;
    rotation_motor_angle_raw = (uint16_t) (rxmsg->data8[0] << 8 | rxmsg->data8[1]);
}

void RoboticArm::set_motor_target_current(int target_current) {
    motor_target_current = target_current;
}

bool RoboticArm::send_motor_target_current() {

    if (!can) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x1FF;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    txmsg.data8[0] = (uint8_t) (rotation_motor_target_current >> 8);
    txmsg.data8[1] = (uint8_t) rotation_motor_target_current;
    txmsg.data8[2] = txmsg.data8[3] = txmsg.data8[4] = txmsg.data8[5] = txmsg.data8[6] = txmsg.data8[7] = 0;

    can->send_msg(&txmsg);
    return true;

}

FetchBulletThread::FetchBulletThread(ioportid_t ioportid, ioportmask_t ioportmask,
                                     uint16_t delay_1, uint16_t delay_2, uint16_t delay_3, CANInterface *can_interface)
        : BaseStaticThread<128>() {
    /**
     * @brief   construct a FetchBullrtThread object
     * @param   ioportid,ioportmask: the GPIO portid connected to  electrically operated valve
     * @param   delay_1: the delay time for the machine hand to hold the box tightly
     * @param   delay_2: the delay time for the machine hand to rotate from inactive position to the position ready to release bullets
     * @param   delay_3: the delay time it takes for bullets to be fell from the box
     * @param   can_interface: the can interface pointer
     */
    _ioportid = ioportid;
    _ioportmask = ioportmask;
    delay1 = delay_1;
    delay2 = delay_2;
    delay3 = delay_3;
    mode = INACTIVE;
    can = can_interface;
}

void FetchBulletThread::fetch_once() {
    /**
     * @brief   make a complete fetch
     */
    if (mode == INACTIVE) {
        mode = MACHINE_HAND_CLAMP;
    }
}

void FetchBulletThread::main(void) {
    // Enable output on the given port
    palSetPadMode(_ioportid, _ioportmask, PAL_MODE_OUTPUT_PUSHPULL);
    // Set the name of this thread
    char name[] = "fetch_bullet";
    setName(name);
    // Monitor the button
    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x1FF;      //TODO: change this
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;   //TODO: There may be a bug, I don't understand can interface very well
    txmsg.data16[0] = (uint16_t) (rotate_current);
    txmsg.data16[1] = txmsg.data16[2] = txmsg.data16[3] = (uint16_t) 0U;
    while (!shouldTerminate()) {
        if (mode == INACTIVE) {
            //hold machine hand to ready for holding the box
            txmsg.data16[0] = (uint16_t) (inactive_current);
            can->send_msg(&txmsg);
            sleep(TIME_MS2I(5));
            continue;
        } else if (mode == MACHINE_HAND_CLAMP) {
            //clamp once, delay delay1 time unit
            palWritePad(_ioportid, _ioportmask, PAL_HIGH);
            sleep(TIME_MS2I(delay1));
            mode = ROTATE;
            continue;
        } else if (mode == ROTATE) {
            //rotate the machine hand
            txmsg.data16[0] = (uint16_t) (rotate_current);
            can->send_msg(&txmsg);
            sleep(TIME_MS2I(delay2));
            mode = RELEASE;
            continue;
        } else if (mode == RELEASE) {
            //release bullet
            txmsg.data16[0] = (uint16_t) (release_current);
            can->send_msg(&txmsg);
            sleep(TIME_MS2I(delay3));   //wait for release
            mode = RESTORE;
            continue;
        } else if (mode == RESTORE) {
            //restore the machine hand to initial state and release the box
            txmsg.data16[0] = (uint16_t) (-rotate_current);
            can->send_msg(&txmsg);
            sleep(TIME_MS2I(delay2));
            txmsg.data16[0] = (uint16_t) inactive_current;
            can->send_msg(&txmsg);
            palWritePad(_ioportid, _ioportmask, PAL_LOW);   //release the box
            mode = INACTIVE;
            continue;
        }
    }
}