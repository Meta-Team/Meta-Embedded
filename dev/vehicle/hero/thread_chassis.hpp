//
// Created by 404 on 2019-05-18.
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

    static constexpr unsigned int chassis_thread_interval = 2; // PID

    static constexpr float COMMON_VX = 1500.0f;  // [mm/s]
    static constexpr float COMMON_VY = 1500.0f;  // [mm/s]
    static constexpr float COMMON_W = 360.0f;    // [degree/s]

    static constexpr float PC_CTRL_RATIO = 0.5f;  // 50% when Ctrl is pressed

    void main() final {

        setName("chassis");

        Chassis::change_pid_params(CHASSIS_PID_V2I_PARAMS);

        while(!shouldTerminate()) {
            if (!StateHandler::remoteDisconnected() && !StateHandler::chassisSeriousErrorOccured()) {

                if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

                    Chassis::calc(0,
                                  -Remote::rc.ch3 * COMMON_VY,
                                  Remote::rc.ch2 * COMMON_W);

                } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                    Chassis::calc(-Remote::rc.ch2 * COMMON_VX,
                                  -Remote::rc.ch3 * COMMON_VY,
                                  Remote::rc.ch2 * COMMON_W);

                } else if (Remote::rc.s1 == Remote::S_DOWN) { //PC control mode

                    // Deterine target velocities



                }
            }
        }
    }
};
#endif //META_INFANTRY_THREAD_CHASSIS_HPP
