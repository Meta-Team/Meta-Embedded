//
// Created by 钱晨 on 10/29/21.
//

#ifndef META_INFANTRY_INSPECTOR_H
#define META_INFANTRY_INSPECTOR_H

#include "can_motor_interface.h"
#include "VirtualCOMPort.h"

class Inspector {
public:
    static void startup_check_VCP();

    static void startup_check_motor();
};


#endif //META_INFANTRY_INSPECTOR_H
