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
};
#endif //META_INFANTRY_THREAD_CHASSIS_HPP
