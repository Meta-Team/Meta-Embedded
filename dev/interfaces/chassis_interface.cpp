//
// Created by liuzikai on 2019-01-12.
//

#include "chassis_interface.h"
#include "common_macro.h"

ChassisInterface::motor_t ChassisInterface::motor[CHASSIS_MOTOR_COUNT];

CANInterface* ChassisInterface::can = nullptr;

bool ChassisInterface::send_chassis_currents() {

    if (!can) return false;

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x200;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    // Fill target currents
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
#if CHASSIS_INTERFACE_ENABLE_CLIP
        ABS_LIMIT(motor[i].target_current, CHASSIS_INTERFACE_MAX_CURRENT);
#endif
        txmsg.data8[i * 2    ] = (uint8_t) (motor[i].target_current >> 8);
        txmsg.data8[i * 2 + 1] = (uint8_t) motor[i].target_current;
    }

    can->send_msg(&txmsg);
    return true;

}