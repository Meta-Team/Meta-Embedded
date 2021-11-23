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
#define THREAD_MOTOR_SKD_PRIO               (HIGHPRIO - 6)
#define THREAD_FEEDBACK_SKD_PRIO            (HIGHPRIO - 7)
#define THREAD_BUTTON_DETECT_PRIO           (HIGHPRIO - 8)
#define THREAD_MOTOR_LG_VISION_PRIO         (HIGHPRIO - 9)

#define THREAD_VIRTUAL_COM_PRIO             (NORMALPRIO)
#define THREAD_INSPECTOR_PRIO               (LOWPRIO + 10)
#define THREAD_SHOOT_BULLET_COUNTER_PRIO    (LOWPRIO + 2)
#define THREAD_SHELL_PRIO                   (LOWPRIO + 1)
#define THREAD_BUZZER_SKD_PRIO              (LOWPRIO)

#endif //META_INFANTRY_THREAD_PRIORITIES_H
