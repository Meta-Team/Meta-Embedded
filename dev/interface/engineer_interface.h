//
// Created by zzb on 2020/1/18.
//

/*
 * This interface includes all four extra motors of engineer: two for the robotic arms and two for the rescue program
 */

#ifndef META_INFANTRY_ENGINEER_IF_H
#define META_INFANTRY_ENGINEER_IF_H

#include "gimbal_interface.h"

class EngineerBase {
    enum motor_id_t {
        RESCUE_LEFT,
        RESCUE_RIGHT,
        ROBOTIC_LEFT,
        ROBOTIC_RIGHT,
        MOTOR_COUNT
    };
};
class EngineerInterface : public EngineerBase {

};


#endif //META_INFANTRY_ENGINEER_INTERFACW_H
