//
// Created by 钱晨 on 11/3/21.
//

#ifndef META_INFANTRY_CONTROLLER_MODULE_H
#define META_INFANTRY_CONTROLLER_MODULE_H

/**
 * @brief Controller modules include Dual Loop PID (for angle control), single loop PID (for velocity control),
 *        direct Input, LQR and MPC controller
 * @usage Create an instance, set the controller type and use calc to get the output.
 * */
class controller_module {
public:
    enum controller_type_t{
        DUAL_PID,
        SINGLE_PID,
        DIRECT,
        LQR,
        MPC
    };
};


#endif //META_INFANTRY_CONTROLLER_MODULE_H
