//
// Created by Anbang Ye on 30/03/2018.
// Revise by liuzikai
//

#ifndef META_INFANTRY_PID_CONTROLLER_H
#define META_INFANTRY_PID_CONTROLLER_H



class PIDController {


public:

    typedef struct {

    } pid_unit_t;

    typedef enum {
        PI, //PID only use P & I
        PID,    //fully PID
    } pid_mode_t;

    PIDController(int _motor_id, pid_mode_t _pid_mode, float _kp, float _ki, float _kd, float _i_limit, float _out_limit)
        : motor_id(_motor_id), pid_mode(_pid_mode), kp(_kp), ki(_ki), kd(_kd), i_limit(_i_limit), out_limit(_out_limit) {

        pid_change(kp, ki, kd, i_limit, out_limit);

        p_out = i_out = d_out = out = 0.0;
        pid_unit.error[0] = pid_unit.error[1] = 0.0;
    }; //initialize pid_controller
    void pid_change(float kp, float ki, float kd, float i_limit, float out_limit);
    float pid_calc(float now, float target);

private:

    int motor_id;   // motor id
    pid_mode_t pid_mode;

    float kp;
    float ki;
    float kd;

    float error[2];

    float p_out;
    float i_out;
    float d_out;

    float i_limit;
    float out_limit;

    float out;

};



#endif //META_INFANTRY_PID_CONTROLLER_H
