//
// Created by liuzikai on 7/21/21.
//

#ifndef META_INFANTRY_KALMAN_FILTER_HPP
#define META_INFANTRY_KALMAN_FILTER_HPP

#include "arm_math.h"
/*
 * Reference: https://github.com/jebohndavida/KalmanFilter
 *
 * It seems that arm_mat_inverse_f32 modifies the source matrix. Some reference:
 *  Comments in https://github.com/jebohndavida/KalmanFilter
 *  https://community.arm.com/developer/tools-software/tools/f/keil-forum/32946/cmsis-dsp-matrix-inverse-problem
 */

/**
 * Kalman filter.
 * @tparam N  Number of variables to keep track of.
 * @tparam M  Number of sensor channels.
 */
template<unsigned N, unsigned M>
class KalmanFilter {
public:

    arm_matrix_instance_f32 x;
    arm_matrix_instance_f32 x_prime;
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 P_prime;
    arm_matrix_instance_f32 F;
    arm_matrix_instance_f32 Q;
    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 H_T;  // H^T
    arm_matrix_instance_f32 K_prime;
    arm_matrix_instance_f32 R;
    arm_matrix_instance_f32 z;

    float32_t x_data[N];
    float32_t x_prime_data[N];
    float32_t P_data[N * N];
    float32_t P_prime_data[N * N];
    float32_t F_data[N * N];
    float32_t Q_data[N * N];
    float32_t H_data[M * N];
    float32_t H_T_data[N * M];
    float32_t K_prime_data[N * M];
    float32_t R_data[M * M];
    float32_t z_data[M];

    void set_H(const float32_t new_H[M * N]) {
        arm_mat_copy(new_H, H);
        arm_mat_trans_f32(&H, &H_T);
    }

private:
    arm_matrix_instance_f32 auxA_N_1;
    arm_matrix_instance_f32 auxA_M_1;
    arm_matrix_instance_f32 auxB_M_1;
    arm_matrix_instance_f32 auxA_N_N;
    arm_matrix_instance_f32 auxB_N_N;
    arm_matrix_instance_f32 auxC_N_N;
    arm_matrix_instance_f32 auxD_N_N;
    arm_matrix_instance_f32 auxA_N_M;
    arm_matrix_instance_f32 auxB_M_N;
    arm_matrix_instance_f32 auxB_M_M;
    arm_matrix_instance_f32 auxC_M_M;
    arm_matrix_instance_f32 auxD_M_M;
    float32_t auxA_N_1_data[N];
    float32_t auxA_M_1_data[M];
    float32_t auxB_M_1_data[M];
    float32_t auxA_N_N_data[N * N];
    float32_t auxB_N_N_data[N * N];
    float32_t auxC_N_N_data[N * N];
    float32_t auxD_N_N_data[N * N];
    float32_t auxA_N_M_data[N * M];
    float32_t auxB_M_N_data[M * N];
    float32_t auxB_M_M_data[M * M];
    float32_t auxC_M_M_data[M * M];
    float32_t auxD_M_M_data[M * M];

public:
    KalmanFilter() {

        reset();

        arm_mat_init_f32(&x, N, 1, x_data);
        arm_mat_init_f32(&x_prime, N, 1, x_prime_data);
        arm_mat_init_f32(&P, N, N, P_data);
        arm_mat_init_f32(&P_prime, N, N, P_prime_data); //
        arm_mat_init_f32(&F, N, N, F_data);
        arm_mat_init_f32(&Q, N, N, Q_data);
        arm_mat_init_f32(&H, M, N, H_data);  // rows: same as z, cols: same as x_prime
        arm_mat_init_f32(&H_T, N, M, H_T_data);
        arm_mat_init_f32(&K_prime, N, M, K_prime_data);
        arm_mat_init_f32(&R, M, M, R_data);
        arm_mat_init_f32(&z, M, 1, z_data);

        arm_mat_init_f32(&auxA_N_1, N, 1, auxA_N_1_data);
        arm_mat_init_f32(&auxA_M_1, M, 1, auxA_M_1_data);
        arm_mat_init_f32(&auxB_M_1, M, 1, auxB_M_1_data);
        arm_mat_init_f32(&auxA_N_N, N, N, auxA_N_N_data);
        arm_mat_init_f32(&auxB_N_N, N, N, auxB_N_N_data);
        arm_mat_init_f32(&auxC_N_N, N, N, auxC_N_N_data);
        arm_mat_init_f32(&auxD_N_N, N, N, auxD_N_N_data);
        arm_mat_init_f32(&auxA_N_M, N, M, auxA_N_M_data);
        arm_mat_init_f32(&auxB_M_N, M, N, auxB_M_N_data);
        arm_mat_init_f32(&auxB_M_M, M, M, auxB_M_M_data);
        arm_mat_init_f32(&auxC_M_M, M, M, auxC_M_M_data);
        arm_mat_init_f32(&auxD_M_M, M, M, auxD_M_M_data);
    }

    void update() {

        /** Prediction Step */

        // x = F * x'
        arm_mat_mult_f32(&F, &x_prime, &x);
        // TODO: + B * u

        // P = F * P' * F^T + Q
        arm_mat_mult_f32(&F, &P_prime, &auxA_N_N);
        arm_mat_trans_f32(&F, &auxB_N_N);
        arm_mat_mult_f32(&auxA_N_N, &auxB_N_N, &auxC_N_N);
        arm_mat_add_f32(&auxC_N_N, &Q, &P);

        /** Update Step */

        // K' = P * H^T / (H * P' * H^T + R);
        arm_mat_mult_f32(&P, &H_T, &auxA_N_M);
        arm_mat_mult_f32(&H, &P, &auxB_M_N);
        arm_mat_mult_f32(&auxB_M_N, &H_T, &auxC_M_M);
        arm_mat_add_f32(&auxC_M_M, &R, &auxB_M_M);
        arm_mat_copy(&auxB_M_M, &auxD_M_M);  // it seems that arm_mat_inverse_f32 modifies source matrix, see top
        arm_mat_inverse_f32(&auxD_M_M, &auxC_M_M);
        arm_mat_mult_f32(&auxA_N_M, &auxC_M_M, &K_prime);

        // x' = x + K' * (z - (H * x))
        arm_mat_mult_f32(&H, &x, &auxA_M_1);
        arm_mat_sub_f32(&z, &auxA_M_1, &auxB_M_1);
        arm_mat_mult_f32(&K_prime, &auxB_M_1, &auxA_N_1);
        arm_mat_add_f32(&x, &auxA_N_1, &x_prime);

        // P' = P - K' * H * P
        arm_mat_mult_f32(&K_prime, &H, &auxB_N_N);
        arm_mat_mult_f32(&auxB_N_N, &P, &auxC_N_N);
        arm_mat_sub_f32(&P, &auxC_N_N, &P_prime);
    }

    void reset() {
        memset(x_data, 0, sizeof(x_data));
        memset(x_prime_data, 0, sizeof(x_prime_data));
        memset(P_data, 0, sizeof(P_data));
        memset(P_prime_data, 0, sizeof(P_prime_data));
        memset(F_data, 0, sizeof(F_data));
        memset(Q_data, 0, sizeof(Q_data));
        memset(H_data, 0, sizeof(H_data));
        memset(H_T_data, 0, sizeof(H_T_data));
        memset(K_prime_data, 0, sizeof(K_prime_data));
        memset(R_data, 0, sizeof(R_data));
        memset(z_data, 0, sizeof(z_data));
    }

private:
    void arm_mat_copy(const arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst) {
        memcpy(dst->pData, src->pData, src->numCols * src->numRows * sizeof(float32_t));
        /*for (unsigned i = 0; i < (src->numCols * src->numRows); i++) {
            dst->pData[i] = src->pData[i];
        }*/
    }

};


#endif //META_INFANTRY_KALMAN_FILTER_HPP
