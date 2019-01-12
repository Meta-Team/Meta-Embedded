//
// Created by liuzikai on 2019-01-12.
//

#ifndef META_INFANTRY_CHASSIS_INTERFACE_H
#define META_INFANTRY_CHASSIS_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

enum chassis_motor_id_t {  // goes in a counter-clockwise order
    CHASSIS_FR, // front right motor, 0
    CHASSIS_FL, // front left motor, 1
    CHASSIS_BL, // back left motor, 2
    CHASSIS_BR, // back right motor, 3
    CHASSIS_MOTOR_COUNT
};

/**
 * Enable clip at the moment of sending current.
 * Only for safety. There is NO signal for clipping. Be sure to eliminate it if more current is needed.
 */
#define CHASSIS_INTERFACE_ENABLE_CLIP                  TRUE

#if CHASSIS_INTERFACE_ENABLE_CLIP
#define CHASSIS_INTERFACE_MAX_CURRENT 5000  // mA
#endif

/**
 * Interface for chassis control
 * NOTICE: set CAN id of each motor the same as chassis_motor_id_t!
 */
class ChassisInterface {

public:

    /**
     * Structure for each motor
     */
    struct motor_t {

        chassis_motor_id_t id;

        // +: ??, -: ??
        // TODO: determine the direction of motor with positive and negative currents
        int target_current;

    };

    static motor_t motor[];

    /**
     * @brief send all target currents
     * @return
     */
    static bool send_chassis_currents();

    /**
     * @brief set CAN interface for receiving and sending
     * @param can_interface
     */
    static void set_can_interface (CANInterface* can_interface) {
        can = can_interface;
    }

private:

    static CANInterface* can;

};


#endif //META_INFANTRY_CHASSIS_INTERFACE_H
