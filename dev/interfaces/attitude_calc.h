#ifndef MATA_ATTITUDE_CALC_H
#define MATA_ATTITUDE_CALC_H

struct quaternion_t {
    float q0, q1, q2, q3;
} quaternion_output;

class BoardAttitude {

public:

    quaternion_t return_quaternion();
    // init pid
    BoardAttitude(int p, int i, int d): kp(p), ki(i), kd(d) {
        BoardAttitude();
    }
    
private:
    // input
    float w[3];
    float a[3];

    // quaternion
    float q[4];

    // integral for pid
    float integral[3];
    // pid constant
    int kp;
    int ki;
    int kd;

    // init q, b
    BoardAttitude() {
        // init q[3]
        q[0] = 1.0f;
        q[1] = q[2] = q[3] = 0.0f;
        // init integral[3]
        integral[0] = integral[1] = integral[2] = 0.0f;
    };
    // update quaternion
    void quaternion_update();
};


#endif // MATA_ATTITUDE_CALC_H