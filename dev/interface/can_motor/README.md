# USE CAN motor controller

## Initiate

### Set up `can_motor_config`

1. Editing CMake target. Ensure `can_motor_config.h` and `can_motor_config.cpp` is in your `target_include_directories`
2. Editing the `can_motor_config.h` and `can_motor_config.cpp`
Please create a c++ class called `CANMotorCFG` in the `.h` file.
a class template is provided below:
   ```c++
   class CANMotorCFG {
       enum             motor_id_t {...,..., MOTOR_COUNT}; // Your logical motor id
       CANMotorBase     CANMotorProfile[MOTOR_COUNT];
       pid_params_t     a2vParams[MOTOR_COUNT];
       pid_params_t     v2iParams[MOTOR_COUNT];
       bool             enable_a2v[MOTOR_COUNT];
       bool             enable_v2i[MOTOR_COUNT];
   }
   ```
   The `CANMotorBase` is a structure that describe CAN motors' hardware information, include
   their SID(Identifier field-ID), CAN channel (usually `can_channel_1` and `can_channel_2`) 
   are available, motor type, initial angle, etc.

   **`enable_a2v` and `enable_v2i` bool should be all false.** Their value should be operated by scheduler-level programs.
   
   A single `CANMotorBase` structure is shown below:
   ```c++
   CANMotorBase {[CANMotorProfile::can_channel_t] can_channel,
                 [int]                            CAN_SID,
                 [CANMotorBase::motor_type_t]     motor_type,
                 [int]                            initial_encoder_angle
   ```
### Set up CAN-BUS
Outside `main()` function, create two `CANInterface` classes with parameters `&CAND1` and `&CAND2`.
   ```c++
   CANInterface can1(&CAND1);
   CANInterface can2(&CAND2)
   ```
   In `main()` function, initiate two `CANInterface` by calling
   ```c++
   can1.start(<CAN1_RX_PRIO>);
   can2.start(<CAN2_RX_PRIO>);
   ```
   where `<CAN1_RX_PRIO>` and `<CAN1_RX_PRIO>` should be replaced with your own thread priorities
### Setup motor controller
   Initiate `CANMotorController` by calling
   ```c++
   CANMotorController::start(<MOTOR_SKD_PRIO>, <FEEDBACK_SKD_PRIO>, &can1, &can2);
   ```
   where `<MOTOR_SKD_PRIO>` and `<FEEDBACK_SKD_PRIO>` should be replaced with your own thread priorities.

## Included functions
See the inline DOXYGEN document for detailed description.




## Further optimization of motor controller
Please finish the requirements below in the future updates.

Current program use dual-loop PID controllers to achieve angle and angular velocity control. In the future, we planned to
modularize controllers and add a configuration file to describe the controller network. A Simulink like UI could be created.