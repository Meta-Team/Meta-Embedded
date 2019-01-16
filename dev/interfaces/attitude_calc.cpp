#include "attitude_calc.h"

#include <cmath>
#include "mpu6500.h"
// TODO: #include "ist8310.h"

quaternion_t BoardAttitude::return_quaternion() {

    // input
    w[0] = MPU6500Controller::angle_speed.x;
    w[1] = MPU6500Controller::angle_speed.y;
    w[2] = MPU6500Controller::angle_speed.z;
    a[0] = MPU6500Controller::a_component.x;
    a[1] = MPU6500Controller::a_component.y;
    a[2] = MPU6500Controller::a_component.z;
    // TODO: fix dt
    dt = MPU6500Controller::dt;
    // TODO: copy the data from ist8310
    // m[0] = 
    // m[1] = 
    // m[2] = 

    quaternion_update();
    
    // return q
    quaternion_output.q0 = q[0];
    quaternion_output.q1 = q[1];
    quaternion_output.q2 = q[2];
    quaternion_output.q3 = q[3];
    return quaternion_output;
}

void BoardAttitude::quaternion_update() {  
    if (!(m[0] == 0.0f && m[1] == 0.0f && m[2] == 0.0f)) {
        // magnetic field can never be zero
        without_ist8310();
    } else {
        // accerleration can be 0
        with_ist8310();
    }
 }

void BoardAttitude::with_ist8310() {
    // normalize a
    if (!(a[0] == 0.0f && a[1] == 0.0f && a[2] == 0.0f)) {
        float lena = 1.0f / sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
        a[0] *= lena;
        a[1] *= lena;
        a[2] *= lena;
    }

    // normalize m
    float lenm = 1.0f / sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    m[0] *= lenm;
    m[1] *= lenm;
    m[2] *= lenm;

    // pre-calculation
    float q00 = q[0] * q[0];
    float q01 = q[0] * q[1];
    float q02 = q[0] * q[2];
    float q03 = q[0] * q[3];
    float q11 = q[1] * q[1];
    float q12 = q[1] * q[2];
    float q13 = q[1] * q[3];
    float q22 = q[2] * q[2];
    float q23 = q[2] * q[3];
    float q33 = q[3] * q[3];

    // reference direction
    // hx and hy are used to calc bx, bz
    float hx = 2.0f * (m[0] * (0.5f - q22 - q33) + m[1] * (q12 - q03) + m[2] * (q13 + q02));
    float hy = 2.0f * (m[1] * (0.5f - q11 - q33) + m[0] * (q12 + q03) + m[2] * (q23 - q01));
    float bx = sqrt(hx*hx + hy*hy);
    //    by = 0;
    float bz = 2.0f * (m[2] * (0.5f - q11 - q22) + m[0] * (q13 - q02) + m[1] * (q23 + q01));

    // calc the direction of gravity
    float halfvx = q13 - q02;
    float halfvy = q01 + q23;
    float halfvz = q00 + q33 - 0.5f;

    // calc the dirction of magnetic field
    float halfwx = bx * (0.5f - q22 - q33) + bz * (q13 - q02);
    float halfwy = bx * (q12 - q03) + bz * (q01 + q23); 
    float halfwz = bz * (0.5f - q11 - q22) + bx * (q02 + q13);

    // calc error
    float halfex = (a[1] * halfvz - a[2] * halfvy) + (m[1] * halfwz - m[2] * halfwy);
    float halfey = (a[2] * halfvx - a[0] * halfvz) + (m[2] * halfwx - m[0] * halfwz);
    float halfez = (a[0] * halfvy - a[1] * halfvx) + (m[0] * halfwy - m[1] * halfwx);

    // correct w using ki
    if (ki > 0.0f) {
        integral[0] = ki * 2.0f * halfex * dt;
        integral[1] = ki * 2.0f * halfey * dt;
        integral[2] = ki * 2.0f * halfez * dt;

        w[0] += integral[0];
        w[1] += integral[1];
        w[2] += integral[2];
    } else {
        integral[0] = integral[1] = integral[2] = 0.0f;
    }

    // correct w using kp
    w[0] += kp * 2 * halfex;
    w[1] += kp * 2 * halfey;
    w[2] += kp * 2 * halfez;

    w[0] *= 0.5f * dt;
    w[1] *= 0.5f * dt;
    w[2] *= 0.5f * dt;

    // update q
    float qa = q[0];
    float qb = q[1];
    float qc = q[2]; 
    float qd = q[3];
    q[0] += (- qb * w[0] - qc * w[1] - qd * w[2]);
    q[1] += (  qa * w[0] + qc * w[2] - qd * w[1]);
    q[2] += (  qa * w[1] - qb * w[2] + qd * w[0]);
    q[3] += (  qa * w[2] + qb * w[1] - qc * w[0]);

    // normalize q
    if (!(q[0] == 0.0f && q[1] == 0.0f && q[2] == 0.0f && q[3] == 0.0f)) {
    float lenq = 1 / sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= lenq;
    q[1] *= lenq;
    q[2] *= lenq;
    q[3] *= lenq;
    }
}

void BoardAttitude::without_ist8310() {
    
    // calc the direction of gravity
    float halfvx = q[1] * q[3] - q[0] * q[2];
    float halfvy = q[0] * q[1] + q[2] * q[3];
    float halfvz = q[0] * q[0] + q[3] * q[3] - 0.5f;

    // calc error
    float halfex = a[1] * halfvz - a[2] * halfvy;
    float halfey = a[2] * halfvx - a[0] * halfvz;
    float halfez = a[0] * halfvy - a[1] * halfvx;

    // correct w using ki
    if (ki > 0.0f) {
        integral[0] = ki * 2.0f * halfex * dt;
        integral[1] = ki * 2.0f * halfey * dt;
        integral[2] = ki * 2.0f * halfez * dt;

        w[0] += integral[0];
        w[1] += integral[1];
        w[2] += integral[2];
    } else {
        integral[0] = integral[1] = integral[2] = 0.0f;
    }

    // correct w using kp
    w[0] += kp * 2 * halfex;
    w[1] += kp * 2 * halfey;
    w[2] += kp * 2 * halfez;
    w[0] *= 0.5f * dt;
    w[1] *= 0.5f * dt;
    w[2] *= 0.5f * dt;

    // update q
    float qa = q[0];
    float qb = q[1];
    float qc = q[2]; 
    float qd = q[3];
    q[0] += (- qb * w[0] - qc * w[1] - qd * w[2]);
    q[1] += (  qa * w[0] + qc * w[2] - qd * w[1]);
    q[2] += (  qa * w[1] - qb * w[2] + qd * w[0]);
    q[3] += (  qa * w[2] + qb * w[1] - qc * w[0]);

    // normalize q
    if (!(q[0] == 0.0f && q[1] == 0.0f && q[2] == 0.0f && q[3] == 0.0f)) {
        float lenq = 1 / sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        q[0] *= lenq;
        q[1] *= lenq;
        q[2] *= lenq;
        q[3] *= lenq;
    }
}