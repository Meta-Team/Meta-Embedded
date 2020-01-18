//
// Created by zzb on 2020/1/18.
//

/*
 * This interface includes all four extra motors of engineer: two for the robotic arms and two for the rescue program
 */

#ifndef META_INFANTRY_ENGINEER_IF_H
#define META_INFANTRY_ENGINEER_IF_H

#include "can_interface.h"

class EngineerBase {
public:
    enum motor_id_t {
        RESCUE_LEFT,
        RESCUE_RIGHT,
        ROBOTIC_LEFT,
        ROBOTIC_RIGHT,
        MOTOR_COUNT
    };
};

class EngineerInterface : public EngineerBase {
public:
    static float present_angle[MOTOR_COUNT];
    static float present_velocity[MOTOR_COUNT];
    static float target_current[MOTOR_COUNT];

    void init();
    int data_to_can();

private:
    static CANInterface * can;

    static void process_feedback_arm();
    static void process_feedback_rescue();

    friend class CANInterface;
};


#endif //META_INFANTRY_ENGINEER_INTERFACW_H
