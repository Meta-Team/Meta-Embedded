#include "attitude_calc.h"

#include "math.h"
#include "mpu6500.h"
// TODO: #include "ist8310.h"

#define PI 3.14159265358979323846f

void BoardAttitude::return_quaternion() {

    MPU6500Controller::getData();
    // input
    w.x = MPU6500Controller::angle_speed.x * PI / 180.0f;
    w.y = MPU6500Controller::angle_speed.y * PI / 180.0f;
    w.z = MPU6500Controller::angle_speed.z * PI / 180.0f;
    a.x = MPU6500Controller::a_component.x;
    a.y = MPU6500Controller::a_component.y;
    a.z = MPU6500Controller::a_component.z;

    dt = MPU6500Controller::dt;
    // TODO: copy the data from ist8310
    m.x = 0;
    m.y = 0;
    m.z = 0;

    quaternion_update();
}

void BoardAttitude::quaternion_update() {  
    if (!(m.x == 0.0f && m.y == 0.0f && m.z == 0.0f)) {
        // magnetic field can never be zero
        without_ist8310();
    } else {
        // accerleration can be 0
        with_ist8310();
    }
 }

void BoardAttitude::with_ist8310() {
    // normalize a
    if (!(a.x == 0.0f && a.y == 0.0f && a.z == 0.0f)) {
        float lena = 1.0f / sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
        a.x *= lena;
        a.y *= lena;
        a.z *= lena;
    }

    // normalize m
    float lenm = 1.0f / sqrt(m.x*m.x + m.y*m.y + m.z*m.z);
    m.x *= lenm;
    m.y *= lenm;
    m.z *= lenm;

    // pre-calculation
    float q00 = q.q0 * q.q0;
    float q01 = q.q0 * q.q1;
    float q02 = q.q0 * q.q2;
    float q03 = q.q0 * q.q3;
    float q11 = q.q1 * q.q1;
    float q12 = q.q1 * q.q2;
    float q13 = q.q1 * q.q3;
    float q22 = q.q2 * q.q2;
    float q23 = q.q2 * q.q3;
    float q33 = q.q3 * q.q3;

    // reference direction
    // hx and hy are used to calc bx, bz
    float hx = 2.0f * (m.x * (0.5f - q22 - q33) + m.y * (q12 - q03) + m.z * (q13 + q02));
    float hy = 2.0f * (m.y * (0.5f - q11 - q33) + m.x * (q12 + q03) + m.z * (q23 - q01));
    float bx = sqrt(hx*hx + hy*hy);
    //    by = 0;
    float bz = 2.0f * (m.z * (0.5f - q11 - q22) + m.x * (q13 - q02) + m.y * (q23 + q01));

    // calc the direction of gravity
    float halfvx = q13 - q02;
    float halfvy = q01 + q23;
    float halfvz = q00 + q33 - 0.5f;

    // calc the dirction of magnetic field
    float halfwx = bx * (0.5f - q22 - q33) + bz * (q13 - q02);
    float halfwy = bx * (q12 - q03) + bz * (q01 + q23); 
    float halfwz = bz * (0.5f - q11 - q22) + bx * (q02 + q13);

    // calc error
    float halfex = (a.y * halfvz - a.z * halfvy) + (m.y * halfwz - m.z * halfwy);
    float halfey = (a.z * halfvx - a.x * halfvz) + (m.z * halfwx - m.x * halfwz);
    float halfez = (a.x * halfvy - a.y * halfvx) + (m.x * halfwy - m.y * halfwx);

    // correct w using ki
    if (ki > 0.0f) {
        integral.x = ki * 2.0f * halfex * dt;
        integral.y = ki * 2.0f * halfey * dt;
        integral.z = ki * 2.0f * halfez * dt;

        w.x += integral.x;
        w.y += integral.y;
        w.z += integral.z;
    } else {
        integral.x = integral.y = integral.z = 0.0f;
    }

    // correct w using kp
    w.x += kp * 2 * halfex;
    w.y += kp * 2 * halfey;
    w.z += kp * 2 * halfez;

    w.x *= 0.5f * dt;
    w.y *= 0.5f * dt;
    w.z *= 0.5f * dt;

    // update q
    float qa = q.q0;
    float qb = q.q1;
    float qc = q.q2; 
    float qd = q.q3;
    q.q0 += (- qb * w.x - qc * w.y - qd * w.z);
    q.q1 += (  qa * w.x + qc * w.z - qd * w.y);
    q.q2 += (  qa * w.y - qb * w.z + qd * w.x);
    q.q3 += (  qa * w.z + qb * w.y - qc * w.x);

    // normalize q
    if (!(q.q0 == 0.0f && q.q1 == 0.0f && q.q2 == 0.0f && q.q3 == 0.0f)) {
    float lenq = 1 / sqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
    q.q0 *= lenq;
    q.q1 *= lenq;
    q.q2 *= lenq;
    q.q3 *= lenq;
    }
}

void BoardAttitude::without_ist8310() {
    
    // calc the direction of gravity
    float halfvx = q.q1 * q.q3 - q.q0 * q.q2;
    float halfvy = q.q0 * q.q1 + q.q2 * q.q3;
    float halfvz = q.q0 * q.q0 + q.q3 * q.q3 - 0.5f;

    Vector3D halfa_e(q.q1 * q.q3 - q.q0 * q.q2, q.q0 * q.q1 + q.q2 * q.q3, q.q0 * q.q0 + q.q3 * q.q3 - 0.5f);

    // calc error
    Vector3D halferr = halfa_e.crossMultiply(a);

    // correct w using ki
    if (ki > 0.0f) {
        integral.x = ki * 2.0f * halferr.x * dt;
        integral.y = ki * 2.0f * halferr.y * dt;
        integral.z = ki * 2.0f * halferr.z * dt;

        w.x += integral.x;
        w.y += integral.y;
        w.z += integral.z;
    } else {
        integral.x = integral.y = integral.z = 0.0f;
    }

    // correct w using kp
    w.x += kp * 2 * halferr.x;
    w.y += kp * 2 * halferr.y;
    w.z += kp * 2 * halferr.z;
    w.x *= 0.5f * dt;
    w.y *= 0.5f * dt;
    w.z *= 0.5f * dt;

    // update q
    float qa = q.q0;
    float qb = q.q1;
    float qc = q.q2; 
    float qd = q.q3;
    q.q0 += (- qb * w.x - qc * w.y - qd * w.z);
    q.q1 += (  qa * w.x + qc * w.z - qd * w.y);
    q.q2 += (  qa * w.y - qb * w.z + qd * w.x);
    q.q3 += (  qa * w.z + qb * w.y - qc * w.x);

    // normalize q
    if (!(q.q0 == 0.0f && q.q1 == 0.0f && q.q2 == 0.0f && q.q3 == 0.0f)) {
        float lenq = 1 / sqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
        q.q0 *= lenq;
        q.q1 *= lenq;
        q.q2 *= lenq;
        q.q3 *= lenq;
    }
}