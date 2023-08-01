# USE Damiao Motor controller

## Initiate

### Set up `damiao_motor_config`

1. Editing CMake target. Ensure `damiao_motor_config.h` and `damiao_motor_config.cpp` is in your `target_include_directories`
2. Editing the `damiao_motor_config.h` and `damiao_motor_config.cpp`
Please create a c++ class called `DamiaoMotorCFG` in the `.h` file.
a class template is provided below:
   ```c++
   class DamiaoMotorBase{
   public:
   CANDriver* can_driver;
   int        masterID;
   int        slaveID;
   float      mitKp;
   float      mitKd;
   float      V_max;   // maximum rotation speed. Unit is Rad/s.
   float      P_max;   // maximum Position. Unit is Rad.
   float      T_max;   // maximum Torque. Unit is N*m.
   float      initial_encoder_angle;
   motor_mode_t mode;
   float      kp_min;
   float      kp_max;
   float      kd_min;
   float      kd_max;
   };
   static constexpr DamiaoMotorBase motorCfg[MOTOR_COUNT] = {
            {can_channel_1,0x00,0x01,1.0,0.3,30,3.141593,10.0,
             0.0,MIT_MODE,0.0,500.0,0.0,5.0},
            {can_channel_2,0x00,0x01,0.0,0.0,30,3.141593,10.0,
             0.0,VEL_MODE,0.0,500.0,0.0,5.0}
    };
   ```
   The `DamiaoMotorCFG` is a structure that describe CAN motors' hardware information, include
   their SID(Identifier field-ID), CAN channes, etc.
   
   Remember that the parameters, like `kp_min`,`kp_max`,`masterID`,etc. are set by the damiao offical
   tool and can't be edited by the embedded program, but the program needs to use these parameters to calculate
   the correct value of the velocity, position etc. because the data on the CAN Bus are adjusted based on
   these parameters. It is important to check the parameters on the official tool before filling
   `damiao_motor_cfg.h`.
   
   
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
### Setup interface and motor controller
   Initiate `DamiaoMotorController` by calling
   ```c++
   DamiaoMotorController::start(<MOTOR_SKD_PRIO>, <FEEDBACK_SKD_PRIO>, &can1, &can2);
   ```
   where `<MOTOR_SKD_PRIO>` and `<FEEDBACK_SKD_PRIO>` should be replaced with your own thread priorities.
   With this command, `DamiaoMotorIF` will be automatically set up. Then the Damiao motor must be initialized
   by sending the fixed startup command, we can do this by calling:
   ```c++
   DamiaoMotorController::motor_enable(<MOTOR_NAME>);
   ```
   To stop the motor, just use `motor_disable` function.
   ### Damiao Motor Contorl
   There are three modes to control the damiao motor, by translation, is MIT mode, position velocity mode and velocity mode.
   The CAN ID for the MIT is mode is the same as the slave ID of the motor, while the CAN ID of position velocity mode is motor
   plus 0x100, and velocity  mode is plusing 0x200. Please refer the Chinese version of Damiao motor document for more details.

   If you want to use the MIT mode, just call the function:
   ```c++
   void DamiaoMotorController::set_target_MIT(DamiaoMotorCFG::MotorName name,float pos,float vel,float torque)
   ```
   And for velocity mode:
   ```c++
   void DamiaoMotorController::set_target_VEL(DamiaoMotorCFG::MotorName name, float vel)
   ```
   And it is the same for position velocity mode.

   
   
## Included functions
See the inline DOXYGEN document in files for detailed description.

**PLEASE READ INLINE DOXYGEN DOCUMENT OF CAN MOTOR INTERFACE!**
It will help you understand the communication DJI motors' communication protocol and how CAN-BUS works.

### Basic functions for `CANMotorController` and `CANMotorIF`

`DamiaoMotorIF` is the interface for Robomaster motors that use *Control Area Network* (CAN) to communicate with our board.
You can get feedback of motors from the interface, by accessing `DamiaoMotorIF::motor_feedback`.

`CANMotorController` is aimed at controlling the motors. You could set angle, angular velocity or torque current through 
this controller. 




## Further optimization of motor controller
Please finish the requirements below in the future updates.

Current program use dual-loop PID controllers to achieve angle and angular velocity control. In the future, we planned to
modularize controllers and add a configuration file to describe the controller network. A Simulink like UI could be created.

