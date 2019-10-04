#ifndef META_ATTITUDE_CALC_H
#define META_ATTITUDE_CALC_H

#include "ch.hpp"
#include "hal.h"

#include "ahrs_math.hpp"
#include "math.h"

#include "mpu6500.h"
#include "ist8310.h"
#include "ahrs_lib.h"

/**
 * @name AHRSOnBoard
 * @brief Attitude And Heading Reference System using on-board IMU and IST
 * @note This module make use of AHRS component from DJI standard program
 * @usage 1. Call start() to enable MPUOnBoard, ISTOnBoard and AHRS updating thread
 *        2. Make use of data, angle, etc.
 */
class AHRSOnBoard : virtual public AbstractAHRS, public MPUOnBoard, public ISTOnBoard {

public:

    /*

     (public)
     (From AbstractAHRS)
     Vector3D get_angle();  // get board angle [degree]
     time_msecs_t get_ahrs_update_time();  // get last update time from system start [ms]

     (From AbstractMPU)
     Override get_gyro(), get_accel() since rotation is needed
     time_msecs_t get_mpu_update_time();  // get last update time from system start [ms]


     (From AbstractIST)
     Vector3D get_magnet();  // get magnet data [uT]
     time_msecs_t get_ist_update_time();  // get last update time from system start [ms]


     (protected)
     (From AbstractMPU)
     Vector3D gyro;   // Data from gyroscope [deg/s]
     Vector3D accel;  // Data from accelerometer [m/s^2]

     (From AbstractIST)
     Vector3D magnet;  // Magnet data [uT]

     (From AbstractAHRS)
     Vector3D angle;

    */

    /**
     * Start MPUOnBoard, ISTOnBoard and AHRS update thread
     * @param mpuRotationMatrix  3x3 Matrix maps gyro and accel to desired coordinate system
     * @param mpuPrio   priority of MPU updating thread
     * @param istPrio   priority of IST updating thread
     * @param ahrsPrio  priority of AHRS updating thread
     */
    void start(const Matrix33 mpuRotationMatrix, tprio_t mpuPrio, tprio_t istPrio, tprio_t ahrsPrio);

    /**
     * Get data from gyroscope (rotated with installPosition)
     * @return Rotated gyro data from gyroscope [deg/s]
     */
    Vector3D get_gyro() override;

    /**
     * Get data from accelerometer (rotated with installPosition)
     * @return Rotated acceleration data from accelerometer [m/s^2]
     */
    Vector3D get_accel() override;

    AHRSOnBoard() : updateThread(*this) {};

private:

    Matrix33 installPos;

    // Local storage
    Vector3D gyro_;  // angular speed [rad/s]
    Vector3D accel_; // acceleration [m/s^2]
    Vector3D mag_;   // magnetic field [uT]

    void fetch_data();  // helper function to fetch data from MPU6500 and IST8130
    void update();

    float q[4];

    class UpdateThread : public chibios_rt::BaseStaticThread<512> {
    public:
        UpdateThread(AHRSOnBoard& ahrs_) : ahrs(ahrs_) {};

    private:
        AHRSOnBoard& ahrs;
        static constexpr unsigned CALCULATION_INTERVAL = 1; // [ms]
        void main() final;
    } updateThread;

};

#endif // META_ATTITUDE_CALC_H