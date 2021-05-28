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
    static void init(CANInterface *can_interface, tprio_t INIT_THREAD_PRIO);

    /**
     * Feedback info structure
     */
    static CANInterface::cap_feedback_t *feedback;
    
    /**
     * Last update time (ms, from system start)
     */
    static time_msecs_t last_feedback_time;

    /**
     * Set super capacitor maximum input power
     * @param input_power   Super capacitor maximum input power
     */
    static void set_power(float input_power);

private:

    static CANInterface *can_;

    friend CANInterface;

    static CANTxFrame txFrame;

    class SuperCapacitorInitThread : public chibios_rt::BaseStaticThread<256> {
    private:
        int INIT_THREAD_INTERVAL = 120;
        void main() final;
    };

    static SuperCapacitorInitThread superCapacitorInitThread;
};


#endif //META_INFANTRY_SUPER_CAPACITOR_PORT_H
