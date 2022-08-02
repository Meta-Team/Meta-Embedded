//
// Created by Quoke on 3/1/2022.
//

#include "lidar_interface.h"

float LidarIF::dist = 0.0f;

void LidarIF::init(CANInterface *can_interface) {
    can_interface->register_callback(0x003, 0x003, process_feedback);
}

void LidarIF::process_feedback(CANRxFrame const *rxmsg) {
    dist = (float) (rxmsg->data8[1] << 8 | rxmsg->data8[0]);
}
