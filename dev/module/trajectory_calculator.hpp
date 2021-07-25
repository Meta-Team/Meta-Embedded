//
// Created by liuzikai on 7/22/21.
//

#ifndef META_INFANTRY_TRAJECTORY_CALCULATOR_HPP
#define META_INFANTRY_TRAJECTORY_CALCULATOR_HPP

#include "math.h"

namespace Trajectory {

#ifndef GRAV_CONSTANT
#define GRAV_CONSTANT 9.80665f
#endif

#ifndef PI
#define PI               3.14159265358979f
#endif

inline float pow2(float v) { return v * v; }

/**
 * Compensate for gravity.
 * @param pitch          [In] target pitch angle & [Out] compensated pitch angle, + for downward, [deg]
 * @param dist           [In] Target distance [mm]
 * @param bullet_speed   [In] Bullet speed [m/s]
 * @param flight_time    [Out] [ms]
 * @return               Whether the bullet can hit the target
 */
inline bool compensate_for_gravity(float &pitch, float dist, float bullet_speed, float &flight_time) {
    float a = pow2(bullet_speed) / (GRAV_CONSTANT / 1E3);
    float y = dist * -sinf(pitch * PI / 180.0f);  // + for upward [mm]
    float b = pow2(a - y) - pow2(dist);
    if (b < 0) return false;
    float c = a - sqrtf(b);
    pitch = -atanf(c / (dist * cosf(pitch * PI / 180.0f))) * 180.0f / PI;
    flight_time = sqrtf((c - y) / (0.5f * (GRAV_CONSTANT / 1E3)));
    return true;
}

inline Vector3D ypdToXYZ(const Vector3D &ypd) {}

}

#endif //META_INFANTRY_TRAJECTORY_CALCULATOR_HPP
