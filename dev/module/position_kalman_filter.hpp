//
// Created by liuzikai on 7/22/21.
//

#ifndef META_INFANTRY_POSITION_KALMAN_FILTER_H
#define META_INFANTRY_POSITION_KALMAN_FILTER_H

#include "kalman_filter.hpp"

class PositionKalmanFilter : protected KalmanFilter<2, 1> {
public:

    explicit PositionKalmanFilter(float Q_position = 0, float Q_velocity = 0, float R_position = 0)
            : KalmanFilter<2, 1>() {

        reset();
        set_Q_position(Q_position);
        set_Q_velocity(Q_velocity);
        set_R_position(R_position);
    }

    void reset() {
        KalmanFilter<2, 1>::reset();

        F_data[0] = 1;
        F_data[3] = 1;

        H_data[0] = H_T_data[0] = 1;

    }

    void set_R_position(float R_position) { R_data[0] = R_position; }

    void set_Q_velocity(float Q_velocity) { Q_data[3] = Q_velocity; }

    void set_Q_position(float Q_position) { Q_data[0] = Q_position; }

    float get_position() const { return x_prime_data[0]; }

    float get_velocity() const { return x_prime_data[1]; }

    void update(float new_position, float time_delta) {
        F_data[1] = time_delta;
        z_data[0] = new_position;
        KalmanFilter<2, 1>::update();
    }

    void reload_position(float new_position) {
        x_prime_data[0] = new_position;
        // Others not changed
    }
};

#endif //META_INFANTRY_POSITION_KALMAN_FILTER_H
