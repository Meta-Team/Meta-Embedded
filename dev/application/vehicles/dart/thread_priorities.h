//
// Created by Wu Feiyang on 6/26/2023.
//

#ifndef META_INFANTRY_THREAD_PRIORITIES_H
#define META_INFANTRY_THREAD_PRIORITIES_H

/// Thread Priority List
#define THREAD_LED                          (HIGHPRIO - 1)
#define THREAD_ALARM                        (HIGHPRIO - 2)

#define THREAD_GIMBAL_SKD_PRIO              (HIGHPRIO - 3)
#define THREAD_CHASSIS_SKD_PRIO             (HIGHPRIO - 4)


#define THREAD_VIRTUAL_COM_PRIO             (NORMALPRIO)
#define THREAD_SUPERCAP_INIT_PRIO           (NORMALPRIO-1)


#define THREAD_INSPECTOR_PRIO               (LOWPRIO + 10)
#define THREAD_COMMUNICATOR_PRIO            (LOWPRIO + 9)


#endif //META_INFANTRY_THREAD_PRIORITIES_H
