//
// Created by liuzikai on 2019-05-17.
//

#ifndef META_INFANTRY_ELEVATOR_H
#define META_INFANTRY_ELEVATOR_H

#include "ch.hpp"
#include "hal.h"

#include "common_macro.h"

#include "elevator_interface.h"
#include "pid_controller.hpp"

/**
 * @note positive height - chassis lift up
 */
class Elevator : public ElevatorInterface, public PIDControllerBase {

public:

    static void change_pid_params(pid_params_t a2v_params, pid_params_t v2i_params);

    static void calc(float height);

    static float get_height();

    static bool motor_reach_target(motor_id_t motor);

    static constexpr float ANGLE_HEIGHT_RATIO = 458.0f;  // [degree/cm]

private:

    /**
     * @brief calculated target velocity
     * @note middle value. For outside code, only for test.
     */
    static float target_velocity[MOTOR_COUNT];
    static float target_angle[MOTOR_COUNT];

    static PIDController a2v_pid[MOTOR_COUNT];
    static PIDController v2i_pid[MOTOR_COUNT];

    /**
     * Perform calculation from angle to velocity and put result into target_velocity_[]
     * @param actual_angle
     * @param target_angle
     * @note for outside code, only for test.
     */
    static void calc_a2v_(motor_id_t motor, float actual_angle_, float target_angle_);

    /**
     * Perform calculation from velocity to current and put result into target_current[]
     * @param actual_velocity
     * @param target_velocity
     * @note for outside code, only for test.
     */
    static void calc_v2i_(motor_id_t motor, float actual_velocity_, float target_velocity_);


    static void calc_motor_(motor_id_t motor);

    // When height different between two front wheels/two back wheels exceeds, ELEVATOR_UNBALANCE is triggered.
    static constexpr float UNBALANCE_LIMIT = 3;  // [cm]

    // When angle different between actual angle and target angle is in the range, we think it reaches its target.
    static constexpr float STABLE_RANGE = 458.0f;  // [degree], ANGLE_HEIGHT_RATIO * 1 cm = 458.0f

    friend void _cmd_elevator_clear_i_out();
    friend void cmd_elevator_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void cmd_elevator_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]);
    friend class ElevatorDebugThread;

};


#endif //META_INFANTRY_ELEVATOR_H
