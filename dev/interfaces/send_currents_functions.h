//
// Created by Chuhao Feng on 2018/12/7.
//

#ifndef META_INFANTRY_SEND_CURRENTS_FUNCTIONS_H
#define META_INFANTRY_SEND_CURRENTS_FUNCTIONS_H

class Send_Currents{

    public:

    CANTxFrame txmsg_chassis; // create a new data structure only for chassis
    txmsg_chassis.IDE = CAN_IDE_STD; // variable declaration
    txmsg_chassis.SID = 0x200;
    txmsg_chassis.RTR = CAN_RTR_DATA;
    txmsg_chassis.DLC = 0x08;

    void send_chassis_currents(void); // function for sending current signals to the chassis

    CANTxFrame txmsg_gimbal; // create a new data structure only for gimbal
    txmsg_gimbal.IDE = CAN_IDE_STD; // variable declaration
    txmsg_gimbal.SID = 0x1FF;
    txmsg_gimbal.RTR = CAN_RTR_DATA;
    txmsg_gimbal.DLC = 0x08;

    void send_gimbal_currents(void); // function for sending current signals to the gimbal

};

#endif //META_INFANTRY_SEND_CURRENTS_FUNCTIONS_H
