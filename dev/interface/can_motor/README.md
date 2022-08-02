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
   
   A single `CANMotorBase` structure is shown below:
   ```c++
   CANMotorBase {[CANMotorProfile::can_channel_t] can_channel,
                 [int]                            CAN_SID,
                 [CANMotorBase::motor_type_t]     motor_type,
                 [int]                            initial_encoder_angle
   ```
## Included functions