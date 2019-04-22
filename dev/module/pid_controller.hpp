//
// Created by Anbang Ye on 30/03/2018.
// Revise by liuzikai
//

#ifndef META_INFANTRY_PID_CONTROLLER_HPP
#define META_INFANTRY_PID_CONTROLLER_HPP


/**
 * PID controller unit
 */
class PIDController {

public:

    float kp;
    float ki;
    float kd;

    float i_limit;  // limit for integral value
    float out_limit;  // limit for total output

    float out;

    PIDController() {
        change_parameters(0, 0, 0, 0, 0);
        p_out = i_out = d_out = out = 0.0;
        error[0] = error[1] = 0.0;
    }

    /**
     * Constructor for new PID controller unit
     * @param _kp
     * @param _ki
     * @param _kd
     * @param _i_limit
     * @param _out_limit
     */
    PIDController(float _kp, float _ki, float _kd, float _i_limit, float _out_limit) {
        change_parameters(_kp, _ki, _kd, _i_limit, _out_limit);
        p_out = i_out = d_out = out = 0.0;
        error[0] = error[1] = 0.0;
    };

    /**
     * Change PID parameters
     * @param _kp
     * @param _ki
     * @param _kd
     * @param _i_limit
     * @param _out_limit
     */
    void change_parameters(float _kp, float _ki, float _kd, float _i_limit, float _out_limit) {
        kp = _kp;
        ki = _ki;
        kd = _kd;
        i_limit = _i_limit;
        out_limit = _out_limit;
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

        p_out = kp * error[0];
        i_out += ki * error[0];
        d_out = kd * (error[0] - error[1]);

        if (i_out > i_limit) i_out = i_limit;
        if (i_out < -i_limit) i_out = -i_limit;

        out = p_out + i_out + d_out;
        if (out > out_limit) out = out_limit;
        if (out < -out_limit) out = -out_limit;

        return out;

    }

    /**
     * @brief clear i_out, mainly for debug and test
     */
    void clear_i_out() {
        i_out = 0.0f;
    }

private:

    float error[2]; // error[0]: error of this time, error[1]: error of last time

    float p_out;
    float i_out;
    float d_out;

};



#endif //META_INFANTRY_PID_CONTROLLER_HPP
