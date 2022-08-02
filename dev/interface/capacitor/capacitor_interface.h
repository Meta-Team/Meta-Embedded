//
// Created by liuzikai on 2019-07-14.
//

#ifndef META_INFANTRY_CAPACITOR_INTERFACE_H
#define META_INFANTRY_CAPACITOR_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

/**
 * @name CapacitorIF
 * @brief Interface to interact with super capacitor from Liewu Control through CAN.
 * @pre Hardware is connected properly
 *
 * @version 2.0a
 * @usage 1. Call init() function
 *        2. Read feedback (stored in class) / Call set_power() to set maximum input power of super capacitor
 */
class CapacitorIF {

public:

    /**
     * Initialize this interface
     * @param can_interface   Initialized CANInterface
     */
    static void init(CANInterface *can_interface, tprio_t INIT_THREAD_PRIO);

    /**
     * Set super capacitor maximum input power
     * @param input_power   Super capacitor maximum input power
     */
    static void set_power(float input_power);

    /**
     * The feedback input voltage
     */
    static float input_voltage;

    /**
     * The feedback capacitor voltage, indicated the charging status of super capacitor
     */
    static float capacitor_voltage;

    /**
     * The input current of super capacitor
     */
    static float input_current;

    /**
     * The output power of super capacitor
     */
    static float output_power;

    /**
     * Last update time (ms, from system start)
     */
    static time_msecs_t last_feedback_time;

private:

    static CANInterface *can_;

    static CANTxFrame txFrame;

    class SuperCapacitorInitThread : public chibios_rt::BaseStaticThread<256> {
    private:
        int INIT_THREAD_INTERVAL = 120;
        void main() final;
    };

    static SuperCapacitorInitThread superCapacitorInitThread;

    static void process_feedback(CANRxFrame const *rxmsg);
};


#endif //META_INFANTRY_CAPACITOR_INTERFACE_H
