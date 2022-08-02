//
// Created by Quoke on 8/2/2022.
//

#ifndef META_EMBEDDED_SENTRY_CHASSIS_LOGIC_H
#define META_EMBEDDED_SENTRY_CHASSIS_LOGIC_H


class SChassisLG {
public:
    enum mode_t {
        FORCED_RELAX_MODE,
        AUTO_MODE,
        MANUAL_MODE
    };

private:

    constexpr static float WHEEL_CURRICULUM = 55.0f;
};


#endif //META_EMBEDDED_SENTRY_CHASSIS_LOGIC_H
