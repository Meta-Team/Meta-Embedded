//
// Created by liuzikai on 7/22/21.
//

#ifndef META_INFANTRY_TRAJECTORY_CALCULATOR_HPP
#define META_INFANTRY_TRAJECTORY_CALCULATOR_HPP

#include <cmath>
#include "arm_math.h"

namespace Trajectory {

#ifndef GRAV_CONSTANT
#define GRAV_CONSTANT 9.80665f
#endif

#ifndef PI
#define PI               3.14159265358979f
#endif

inline float pow2(float v) { return v * v; }
inline float cos(float x) { return arm_cos_f32(x); }
inline float sin(float x) { return arm_sin_f32(x); }
inline float tan(float x) { return arm_sin_f32(x) / arm_cos_f32(x); }
inline float sqrt(float x) { return sqrtf(x); }
inline float atan(float x) { return atanf(x); }

/**
 * Compensate for gravity.
 * @param pitch          [In] target pitch angle & [Out] compensated pitch angle, + for downward, [deg]
 * @param dist           [In] Target distance [mm]
 * @param bullet_speed   [In] Bullet speed [m/s]
 * @param flight_time    [Out] [ms]
 * @return               Whether the bullet can hit the target
 */
inline bool compensate_for_gravity(float &pitch, float dist, float bullet_speed, float &flight_time) {
    float a = pow2(bullet_speed) / (GRAV_CONSTANT / 1E3f);
    float y = dist * -sin(pitch * PI / 180.0f);  // + for upward [mm]
    float b = pow2(a - y) - pow2(dist);
    if (b < 0) return false;
    float c = a - sqrt(b);
    pitch = -atan(c / (dist * cos(pitch * PI / 180.0f))) * 180.0f / PI;
    flight_time = sqrt((c - y) / (0.5f * (GRAV_CONSTANT / 1E3f)));
    return true;
}

inline Vector3D ypdToXYZ(const Vector3D &ypd) {
    float x = ypd.z * tan(ypd.x * PI / 180.0f);
    float y = ypd.z * tan(ypd.y * PI / 180.0f);
    return {x, y, sqrtf(pow2(ypd.z)) - pow2(x) - pow2(y)};
}

}

#endif //META_INFANTRY_TRAJECTORY_CALCULATOR_HPP
