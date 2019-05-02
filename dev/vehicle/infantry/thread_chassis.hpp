//
// Created by liuzikai on 2019-05-02.
//

#ifndef META_INFANTRY_THREAD_CHASSIS_H
#define META_INFANTRY_THREAD_CHASSIS_H

/**
 * @name ChassisThread
 * @brief thread to control chassis
 * @pre Remote interpreter starts receiving
 * @pre ChassisInterface(Chassis) get properly init()
 */
class ChassisThread : public chibios_rt::BaseStaticThread<1024> {

private:

    static constexpr unsigned int chassis_thread_interval = 2;  // PID calculation interval [ms]

    static constexpr float COMMON_VX = 1500.0f;  // [mm/s]
    static constexpr float COMMON_VY = 1500.0f;  // [mm/s]
    static constexpr float COMMON_W = 360.0f;    // [degree/s]

    static constexpr float PC_CTRL_RATIO = 0.5f;  // 50% when Ctrl is pressed

    void main() final {

        setName("chassis");

        Chassis::change_pid_params(CHASSIS_PID_V2I_PARAMS);

        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

                Chassis::calc(0,
                              -Remote::rc.ch3 * COMMON_VY,
                              Remote::rc.ch2 * COMMON_W);

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                Chassis::calc(-Remote::rc.ch2 * COMMON_VX,
                              -Remote::rc.ch3 * COMMON_VY,
                              Remote::rc.ch0 * COMMON_W);

            } else if (Remote::rc.s1 == Remote::S_DOWN) { // PC control mode

                // Determine target velocities

                float target_vx, target_vy, target_w;

                /** NOTICE: here the minus sign make it incoherent with the initial definition of chassis coordinate */

                if (Remote::key.w) target_vy = -COMMON_VY;
                else if (Remote::key.s) target_vy = COMMON_VY;
                else target_vy = 0;

                if (Remote::key.q) target_vx = -COMMON_VX;
                else if (Remote::key.e) target_vx = COMMON_VX;
                else target_vx = 0;

                if (Remote::key.a) target_w = -COMMON_W;
                else if (Remote::key.d) target_w = COMMON_W;
                else target_w = 0;

                if (Remote::key.ctrl) {
                    target_vx *= PC_CTRL_RATIO;
                    target_vy *= PC_CTRL_RATIO;
                    target_w *= PC_CTRL_RATIO;
                }

                Chassis::calc(target_vx, target_vy, target_w);

            } else {

                for (int i = 0; i < Chassis::CHASSIS_MOTOR_COUNT; i++) {
                    Chassis::target_current[i] = 0;
                }

            }

            Chassis::send_chassis_currents();

            sleep(TIME_MS2I(chassis_thread_interval));

        }
    }

};

#endif //META_INFANTRY_THREAD_CHASSIS_H
