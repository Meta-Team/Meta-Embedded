//
// Created by zzb on 2020/1/18.
//

/*
 * This interface includes all four extra motors of engineer: two for the robotic arms and two for the rescue program
 */

#ifndef META_INFANTRY_ENGINEER_IF_H
#define META_INFANTRY_ENGINEER_IF_H

#include "can_interface.h"
#include "air_tank_interface.h"

#define ENGR_INTERFACE_ENABLE_CLIP  FALSE
#if ENGR_INTERFACE_ENABLE_CLIP
#define ENGR_RESCUE_INTERFACE_MAX_CURRENT 5000
#define ENGR_ROBOTIC_INTERFACE_MAX_CURRENT 5000
#endif

enum motor_id_t {
    RESCUE_LEFT,
    RESCUE_RIGHT,
    ROBOTIC_LEFT,
    ROBOTIC_RIGHT,
    MOTOR_COUNT
};

class EngineerInterface{
public:
    enum motor_can_channel_t {
        none_can_channel,
        can_channel_1,
        can_channel_2
    };

    struct motor_can_config_t {
        motor_can_channel_t motor_can_channel;
        unsigned motor_can_id;
        CANInterface::motor_type_t motor_type;
    };

    static void init( CANInterface *can1_interface,                      CANInterface *can2_interface,
                      motor_can_config_t motor_can_config[MOTOR_COUNT]);

    /**
 * Motor feedback structure
 */
    static CANInterface::motor_feedback_t *feedback[MOTOR_COUNT];


    /**
     * Target current array in the order defined in motor_id_t
     * This is an array of pointers whose address are from CanInterface::processFeedbackThread.feedback
     * We save the array of pointers in engineer interface, so we can read these feedback data through pointers
     */
    static int *target_current[MOTOR_COUNT];

    /**
     * Send target_current of each motor
     * @return Whether currents are sent successfully
     */
    static void enable_engr_current_clip();

    /**
     * Several state for Air tank control?
     * TODO: Maybe move these methods to the scheduler.
     */
    static int door_state;
    static void change_door();
    static void pop_ammo_bin();

private:

    static CANInterface *can1_;
    static CANInterface *can2_;

    friend class CANInterface;
};


#endif //META_INFANTRY_ENGINEER_INTERFACW_H
