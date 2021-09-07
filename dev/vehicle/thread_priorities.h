//
// Created by liuzikai on 7/29/21.
//

#ifndef META_INFANTRY_THREAD_PRIORITIES_H
#define META_INFANTRY_THREAD_PRIORITIES_H

/// Thread Priority List
#define THREAD_CAN1_RX_PRIO                 (HIGHPRIO - 1)
#define THREAD_CAN1_TX_PRIO                 (HIGHPRIO - 2)
#define THREAD_CAN2_RX_PRIO                 (HIGHPRIO - 3)
#define THREAD_CAN2_TX_PRIO                 (HIGHPRIO - 4)
#define THREAD_AHRS_PRIO                    (HIGHPRIO - 5)
#define THREAD_GIMBAL_SKD_PRIO              (HIGHPRIO - 6)
#define THREAD_VISION_SKD_PRIO              (HIGHPRIO - 7)
#define THREAD_SHOOT_SKD_PRIO               (HIGHPRIO - 8)
#define THREAD_CHASSIS_SKD_PRIO             (HIGHPRIO - 9)
#define THREAD_GIMBAL_LG_VISION_PRIO        (HIGHPRIO - 10)
#define THREAD_SHOOT_LG_VISION_PRIO         (HIGHPRIO - 11)
#define THREAD_REFEREE_SENDING_PRIO         (NORMALPRIO + 3)
#define THREAD_REFEREE_SKD_PRIO             (NORMALPRIO + 2)
#define THREAD_GIMBAL_LG_SENTRY_PRIO        (NORMALPRIO + 1)
#define THREAD_USER_PRIO                    (NORMALPRIO)
#define THREAD_USER_ACTION_PRIO             (NORMALPRIO - 1)
#define THREAD_CHASSIS_LG_DODGE_PRIO        (NORMALPRIO - 2)
#define THREAD_CHASSIS_POWER_SET_PRIO       (NORMALPRIO - 3)
#define THREAD_SHOOT_LG_STUCK_DETECT_PRIO   (NORMALPRIO - 4)
#define THREAD_SUPERCAP_INIT_PRIO           (NORMALPRIO - 5)
#define THREAD_REFEREE_LD_PRIO              (NORMALPRIO - 6)
#define THREAD_INSPECTOR_PRIO               (LOWPRIO + 10)
#define THREAD_SHOOT_BULLET_COUNTER_PRIO    (LOWPRIO + 2)
#define THREAD_SHELL_PRIO                   (LOWPRIO + 1)
#define THREAD_BUZZER_SKD_PRIO              (LOWPRIO)

#endif //META_INFANTRY_THREAD_PRIORITIES_H
