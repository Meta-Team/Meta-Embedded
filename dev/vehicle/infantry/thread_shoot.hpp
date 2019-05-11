//
// Created by liuzikai on 2019-05-02.
//

#ifndef META_INFANTRY_THREAD_SHOOT_HPP
#define META_INFANTRY_THREAD_SHOOT_HPP

/**
 * @name ShootThread
 * @brief Thread to control shooter
 * @pre Remote interpreter starts receiving
 * @pre MPU6500 starts updating
 * @pre GimbalInterface(Shoot) get properly init()
 * @pre GimbalThread starts
 */
class ShootThread : public chibios_rt::BaseStaticThread<1024> {

    static constexpr unsigned int SHOOT_THREAD_INTERVAL = 5; // PID calculation interval [ms]

    static constexpr float COMMON_SHOOT_SPEED = 5;

    bool pc_right_pressed = false;  // local variable to control one click of right button of mouse

    // TODO: write bullet count control

    void main() final {

        setName("shoot");

        Shoot::change_pid_params(GIMBAL_PID_BULLET_LOADER_V2I_PARAMS);

        while (!shouldTerminate()) {


            if (!StateHandler::remoteDisconnected() && !StateHandler::gimbalSeriousErrorOccured()) {
                if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

                    // Friction wheels
                    Shoot::set_friction_wheels(GIMBAL_REMOTE_FRICTION_WHEEL_DUTY_CYCLE);

                    // Bullet loader motor
                    if (Remote::rc.ch1 > 0.5) {
                        Shoot::calc_bullet_loader(COMMON_SHOOT_SPEED);
                    } else {
                        Shoot::calc_bullet_loader(0);
                    }

                } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

                    // Friction wheels
                    Shoot::set_friction_wheels(GIMBAL_REMOTE_FRICTION_WHEEL_DUTY_CYCLE);

                    // Bullet loader motor
                    if (Remote::rc.ch3 < 0.1) {
                        Shoot::calc_bullet_loader(Remote::rc.ch3 * COMMON_SHOOT_SPEED);
                    } else {
                        Shoot::calc_bullet_loader(0);
                    }

                } else if (Remote::rc.s1 == Remote::S_DOWN) { // PC control mode

                    // Bullet loader motor
                    if (Remote::mouse.press_left) {

                        if (Shoot::fw_duty_cycle == 0) {  // if fw have not start, start it and delay for shooting
                            Shoot::set_friction_wheels(GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE);
                            sleep(TIME_MS2I(500));
                        }

                        Shoot::calc_bullet_loader(COMMON_SHOOT_SPEED);

                    } else {

                        Shoot::calc_bullet_loader(0);
                    }

                    // Friction wheels
                    if (Remote::mouse.press_right) {
                        if (!pc_right_pressed) {
                            if (Shoot::fw_duty_cycle == 0) {
                                Shoot::set_friction_wheels(GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE);
                            } else {
                                Shoot::set_friction_wheels(0);
                            }
                            pc_right_pressed = true;
                        }
                    } else {
                        if (pc_right_pressed) {
                            pc_right_pressed = false;
                        }
                    }

                } else {

                    Shoot::set_friction_wheels(0);
                    Shoot::target_current[Shoot::BULLET] = 0;

                }

            } else {  // StateHandler::remoteDisconnected() || StateHandler::gimbalSeriousErrorOccured()

                Shoot::set_friction_wheels(0);
                Shoot::target_current[Shoot::BULLET] = 0;

            }

            // There is no need to send current here. It's handle by GimbalThread.

            sleep(TIME_MS2I(SHOOT_THREAD_INTERVAL));
        }
    }
};

#endif //META_INFANTRY_THREAD_SHOOT_HPP
