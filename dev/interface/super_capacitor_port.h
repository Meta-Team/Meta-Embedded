//
// Created by liuzikai on 2019-07-14.
//

#ifndef META_INFANTRY_SUPER_CAPACITOR_PORT_H
#define META_INFANTRY_SUPER_CAPACITOR_PORT_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

/**
 * @name SuperCapacitor
 * @brief Interface to interact with super capacitor from Liewu Control through CAN.
 * @pre Hardware is connected properly
 * @usage 1. Call Call init(CANInterface *) with a properly initialized CANInterface.
 *        2. Read feedback / Call set_power() to set maximum input power of super capacitor
 */
class SuperCapacitor {

public:

    /**
     * Initialize this interface
     * @param can_interface   Initialized CANInterface
     */
    static void init(CANInterface *can_interface);

    /**
     * Feedback info structure
     */
    static struct feedback_t {
        float input_voltage;      // [V]
        float capacitor_voltage;  // [V]
        float input_current;      // [A]
        float output_power;       // [W]
    } feedback;
    
    /**
     * Last update time (ms, from system start)
     */
    static time_msecs_t last_feedback_time;

    /**
     * Set super capacitor maximum input power
     * @param input_power   Super capacitor maximum input power
     */
    void set_power(float input_power);

private:

    static CANInterface *can_;

    static void process_feedback(CANRxFrame const *rxmsg);  // callback function
    friend CANInterface;

};


#endif //META_INFANTRY_SUPER_CAPACITOR_PORT_H
