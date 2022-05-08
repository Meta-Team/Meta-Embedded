//
// Created by Quoke on 3/1/2022.
//

#ifndef META_INFANTRY_LIDAR_INTERFACE_H
#define META_INFANTRY_LIDAR_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

class LidarIF {
public:
    /**
     * Initialize the lidar interface
     */
    static void init(CANInterface *can_interface);

    /**
     * [m] Distance received from lidar
     */
    static float dist;

    /**
     * Process the feedback from super capacitor
     * @param rxmsg
     */
    static void process_feedback(CANRxFrame const *rxmsg);
};


#endif //META_INFANTRY_LIDAR_INTERFACE_H
