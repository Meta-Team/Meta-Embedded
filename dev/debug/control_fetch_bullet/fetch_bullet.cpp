//
// Created by Ye Anbang on 2019/2/2.
//
#include "fetch_bullet.h"
#include "common_macro.h"
using namespace chibios_rt;

static CANTxFrame txmsg;

FetchBulletThread::FetchBulletThread(ioportid_t ioportid, ioportmask_t ioportmask,
        uint16_t delay_1, uint16_t delay_2, uint16_t delay_3, CANInterface *can_interface)
        : BaseStaticThread<128>() {
    _ioportid = ioportid;
    _ioportmask = ioportmask;
    delay1 = delay_1;
    delay2 = delay_2;
    delay3 = delay_3;
    mode = INACTIVE;
    can = can_interface;
}

void FetchBulletThread::fetch_once() {
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
    while(!shouldTerminate()) {
        if (mode == INACTIVE) {
            txmsg.data16[0] = (uint16_t) (inactive_current);
            can->send_msg(&txmsg);
            sleep(TIME_MS2I(5));
            continue;
        }else if (mode == MACHINE_HAND_CLAMP) {
            //clamp once, delay delay1 time unit
            palWritePad(_ioportid, _ioportmask, PAL_HIGH);
            sleep(TIME_MS2I(delay1));
            mode = ROTATE;
            continue;
        }else if (mode == ROTATE){
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
            sleep(TIME_MS2I(delay3));
            mode = RESTORE;
            continue;
        } else if (mode == RESTORE) {
            //restore the machine hand to initial state and release the box
            txmsg.data16[0] = (uint16_t) (-rotate_current);
            can->send_msg(&txmsg);
            sleep(TIME_MS2I(delay2));
            txmsg.data16[0] = (uint16_t) inactive_current;
            can->send_msg(&txmsg);
            palWritePad(_ioportid, _ioportmask, PAL_LOW);
            mode = INACTIVE;
            continue;
        }
    }
}