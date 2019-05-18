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

private:

    bool external_mode = false;
    float external_target_vx = 0.0f;
    float external_target_vy = 0.0f;
    float external_target_w = 0.0f;

public:

    void enable_external_mode() {
        external_mode = true;
    }

    void disable_external_mode() {
        external_mode = false;
    }

    void set_external_target(float target_vx, float target_vy, float target_w) {
        external_target_vx = target_vx;
        external_target_vy = target_vy;
        external_target_w = target_w;
    }

private:

    static constexpr unsigned int CHASSIS_THREAD_INTERVAL = 2;  // [ms]

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

            if (external_mode) {

                Chassis::calc(external_target_vx, external_target_vy, external_target_w);

            } else {

                if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                    Chassis::calc(-Remote::rc.ch2 * 1000.0f,
                                  -Remote::rc.ch3 * 1000.0f,
                                  Remote::rc.ch0 * 180.0f);

                } else if (Remote::rc.s1 == Remote::S_DOWN) {

                    float target_vx = 0, target_vy = 0, target_w = 0;

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

                    Chassis::calc(target_vx, target_vy, target_w);

                } else {

                    for (int i = 0; i < Chassis::MOTOR_COUNT; i++) {
                        Chassis::target_current[i] = 0;
                    }

                }
            }

            Chassis::send_chassis_currents();

            sleep(TIME_MS2I(CHASSIS_THREAD_INTERVAL));

        }
    }
};

#endif //META_INFANTRY_THREAD_CHASSIS_HPP
