//
// Created by liuzikai on 2019-05-18.
//

#ifndef META_INFANTRY_THREAD_CHASSIS_HPP
#define META_INFANTRY_THREAD_CHASSIS_HPP

/**
 * @name ChassisThread
 * @brief thread to control chassis
 * @pre RemoteInterpreter start receive
 * @pre initialize ChassisInterface with CAN driver
 */
class ChassisThread : public chibios_rt::BaseStaticThread<1024> {

    static constexpr unsigned int chassis_thread_interval = 20;

    /**
     * Params for user
     * @note also update ONES doc
     */
    static constexpr float PC_W_VY = -500.0f;
    static constexpr float PC_S_VY = 500.0f;
    static constexpr float PC_E_VX = -500.0f;
    static constexpr float PC_Q_VX = 500.0f;
    static constexpr float PC_A_W = -100.0f;
    static constexpr float PC_D_W = 100.0f;

    static constexpr float PC_CTRL_RATIO = 0.5f;

    void main() final {

        setName("chassis");

        Chassis::change_pid_params({CHASSIS_PID_V2I_PARAMS});

        while (!shouldTerminate()) {

            if ((Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) ||
                (Remote::rc.s1 == Remote::S_DOWN)) {

                float target_vx = 0, target_vy = 0, target_w = 0;

                if (elevatorThread.get_status() == elevatorThread.STOP) {  // if elevator thread is not in action,

                    // let user control the chassis
                    if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                        target_vx = -Remote::rc.ch2 * 1000.0f;
                        target_vy = -Remote::rc.ch3 * 1000.0f;
                        target_w = Remote::rc.ch0 * 180.0f;

                    } else if (Remote::rc.s1 == Remote::S_DOWN) {

                        if (Remote::key.w) target_vy = PC_W_VY;
                        else if (Remote::key.s) target_vy = PC_S_VY;
                        else target_vy = 0;

                        if (Remote::key.q) target_vx = PC_Q_VX;
                        else if (Remote::key.e) target_vx = PC_E_VX;
                        else target_vx = 0;

                        if (Remote::key.a) target_w = PC_A_W;
                        else if (Remote::key.d) target_w = PC_D_W;
                        else target_w = 0;

                        if (Remote::key.ctrl) {
                            target_vx *= PC_CTRL_RATIO;
                            target_vy *= PC_CTRL_RATIO;
                            target_w *= PC_CTRL_RATIO;
                        }
                    } else {
                        target_vx = target_vy = target_w = 0;
                    }

                } else {

                    target_vx = target_w = 0;
                    target_vy = elevatorThread.get_chassis_target_vy();

                }

                // Perform calculation
                Chassis::calc(target_vx, target_vy, target_w);

            } else {

                for (int i = 0; i < Chassis::MOTOR_COUNT; i++) {
                    Chassis::target_current[i] = 0;
                }

            }

            Chassis::send_chassis_currents();
            sleep(TIME_MS2I(chassis_thread_interval));

        }
    }
};

#endif //META_INFANTRY_THREAD_CHASSIS_HPP
