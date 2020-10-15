//
// Created by Anbang Ye on 30/03/2018.
// Revise by liuzikai
//

#ifndef META_INFANTRY_PID_CONTROLLER_HPP
#define META_INFANTRY_PID_CONTROLLER_HPP

class PIDControllerBase {
public:

    struct pid_params_t {
        float kp;
        float ki;
        float kd;

        float i_limit;    // limit for integral value
        float out_limit;  // limit for total output
    };

};

/**
 * PID controller unit
 */
class PIDController : public PIDControllerBase {

public:

    float out;

    float abs_float(float val){
        if (val < 0){
            return (-1*val);
        }
        else{
            return val;
        }
    }

    PIDController() {
        change_parameters({0, 0, 0, 0, 0});
        p_out = i_out = d_out = out = 0.0;
        error[0] = error[1] = 0.0;
    }

    /**
     * Change PID parameters
     * @param p_
     */
    void change_parameters(pid_params_t p_) {
        p.kp = p_.kp;
        p.ki = p_.ki;
        p.kd = p_.kd;
        p.i_limit = p_.i_limit;
        p.out_limit = p_.out_limit;
    }

    pid_params_t get_parameters() {
        return p;
    }

    /**
     * Perform one calculation
     * @param now
     * @param target
     * @return out
     */
    float calc(float now, float target) {

        error[1] = error[0];
        error[0] = target - now;

        p_out = p.kp * error[0];
        i_out += p.ki * error[0];
        d_out = p.kd * (error[0] - error[1]);

        if (i_out > p.i_limit) i_out = p.i_limit;
        if (i_out < -p.i_limit) i_out = -p.i_limit;

        out = p_out + i_out + d_out;
        if (abs_float(error[0]) <= abs_float(target*0.03) && i_clip_enabled){
            i_out = 0.0f;
        }
        if (out > p.out_limit) out = p.out_limit;
        if (out < -p.out_limit) out = -p.out_limit;

        return out;

    }

    /**
     * @brief clear i_out, mainly for debug and test
     */
    void clear_i_out() {
        i_out = 0.0f;
    }

    float get_i_out(){
        return i_out;
    }

    float get_error_0(){
        return error[0];
    }

    void enable_i_clip() {
        i_clip_enabled = true;
    }
private:

    pid_params_t p;

    float error[2];  // error[0]: error of this time, error[1]: error of last time

    float p_out;
    float i_out;
    float d_out;

    bool i_clip_enabled = false;

};



#endif //META_INFANTRY_PID_CONTROLLER_HPP

