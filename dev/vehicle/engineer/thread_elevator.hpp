//
// Created by liuzikai on 2019-05-18.
//

#ifndef META_INFANTRY_THREAD_ELEVATOR_HPP
#define META_INFANTRY_THREAD_ELEVATOR_HPP

class ElevatorThread : public chibios_rt::BaseStaticThread<1024> {

private:

    float front_target_height_ = 0.0f;
    float back_target_height_ = 0.0f;

public:

    void set_front_target_height(float front_target_height) {
        front_target_height_ = front_target_height;
    }

    void set_back_target_height(float back_target_height) {
        back_target_height_ = back_target_height;
    }

private:

    static constexpr unsigned ELEVATOR_THREAD_INTERVAL = 2; // [ms]

    void main() final {

        setName("elevator");

        Elevator::change_pid_params({ELEVATOR_PID_A2V_PARAMS}, {ELEVATOR_PID_V2I_PARAMS});

        while(!shouldTerminate()) {

            Elevator::calc_front(front_target_height_);
            Elevator::calc_back(back_target_height_);

            Elevator::send_elevator_currents();

            sleep(TIME_MS2I(ELEVATOR_THREAD_INTERVAL));
        }

    }

};

#endif //META_INFANTRY_THREAD_ELEVATOR_HPP
