/*
 * imu.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: simonyu
 */

#include "platform/imu.h"
#include "utility/math.h"
#include "common/parameter.h"
#include "platform/serial.h"

namespace biped
{
IMU::IMU() : bmx160_(&Wire)
{
    initializeBMX160();
    initializeMPU6050();
}

IMUData
IMU::getDataBMX160() const
{
    return bmx160_data_;
}

IMUData
IMU::getDataMPU6050() const
{
    return mpu6050_data_;
}

void IMU::readBMX160() {
    sBmx160SensorData_t accel_data, gyro_data, mag_data;
    bmx160_.getAllData(&mag_data, &gyro_data, &accel_data);

    bmx160_data_.acceleration_x = accel_data.x;
    bmx160_data_.acceleration_y = accel_data.y;
    bmx160_data_.acceleration_z = accel_data.z;

    bmx160_data_.attitude_x = gyro_data.x;
    bmx160_data_.attitude_y = gyro_data.y;
    bmx160_data_.attitude_z = gyro_data.z;

    bmx160_data_.compass_x = mag_data.x;
    bmx160_data_.compass_y = mag_data.y;
    bmx160_data_.compass_z = mag_data.z;
}

void
IMU::readMPU6050()
{
    /*
     *  Sensor event structs.
     *
     *  sensors_event_t struct reference:
     *  http://adafruit.github.io/Adafruit_CircuitPlayground/html/structsensors__event__t.html
     */
    sensors_event_t acceleration;
    sensors_event_t angular_velocity;
    sensors_event_t temperature;

    /*
     *  Read from MPU6050 and populate sensor event structs.
     */
    if (!mpu6050_.getEvent(&acceleration, &angular_velocity, &temperature))
    {
        Serial(LogLevel::error) << "Failed to read from MPU6050.";
        return;
    }

    /*
     *  Using the populated sensor event structs, populate
     *  the corresponding entries in the member sensor
     *  data struct.
     *
     *  Note that the raw data read from the MPU6050 might not be in the
     *  standard body reference frame. Refer to the following materials
     *  to correctly convert the raw data into the standard body
     *  reference frame.
     *
     *  Standard body reference frame:
     *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
     *
     *  Rotational right-hand rule:
     *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
     */
    // TODO LAB 5 YOUR CODE HERE.
    mpu6050_data_.acceleration_x = acceleration.acceleration.x;
    mpu6050_data_.acceleration_y = -acceleration.acceleration.y;
    mpu6050_data_.acceleration_z = acceleration.acceleration.z;

    mpu6050_data_.angular_velocity_x = -angular_velocity.gyro.x;
    mpu6050_data_.angular_velocity_y = angular_velocity.gyro.y;
    mpu6050_data_.angular_velocity_z = -angular_velocity.gyro.z;

    calculateAttitudeMPU6050();
}

void
IMU::initializeBMX160()
{
    /*
     *  Initialize BMX160 and validate the initialization.
     */
    if (!bmx160_.begin())
    {
        Serial(LogLevel::error) << "Failed to initialize BMX160.";
        return;
    }

    /*
     *  Configure BMX160.
     */
    bmx160_.setAccelRange(eAccelRange_2G);
    bmx160_.setGyroRange(eGyroRange_125DPS);

    /*
     *  Perform initial BMX160 read.
     */
    readBMX160();

    /*
     *  Perform initial attitude calculation.
     *  See calculateAttitude function first for details.
     */
    calculateAttitudeBMX160();

    /*
     *  Configure Y attitude Kalman filter.
     */
    bmx160_kalman_filter_attitude_y_.setAngle(radiansToDegrees(bmx160_data_.attitude_y));
    bmx160_kalman_filter_attitude_y_.setQangle(KalmanFilterParameter::q_angle);
    bmx160_kalman_filter_attitude_y_.setQbias(KalmanFilterParameter::q_bias);
    bmx160_kalman_filter_attitude_y_.setRmeasure(KalmanFilterParameter::r_measure);
}

void
IMU::initializeMPU6050()
{
    /*
     *  Initialize MPU6050 and validate the initialization.
     */
    if (!mpu6050_.begin(AddressParameter::imu_mpu6050, &Wire, 0))
    {
        Serial(LogLevel::error) << "Failed to initialize MPU6050.";
        return;
    }

    /*
     *  Configure MPU6050.
     */
    mpu6050_.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu6050_.setGyroRange(MPU6050_RANGE_250_DEG);

    /*
     *  Perform initial MPU6050 read.
     */
    // TODO LAB 5 YOUR CODE HERE.
    readMPU6050();

    /*
     *  Perform initial attitude calculation.
     *  See calculateAttitude function first for details.
     */
    // TODO LAB 5 YOUR CODE HERE.
    calculateAttitudeMPU6050();
    /*
     *  Configure Y attitude Kalman filter.
     */
    mpu6050_kalman_filter_attitude_y_.setAngle(radiansToDegrees(mpu6050_data_.attitude_y));
    mpu6050_kalman_filter_attitude_y_.setQangle(KalmanFilterParameter::q_angle);
    mpu6050_kalman_filter_attitude_y_.setQbias(KalmanFilterParameter::q_bias);
    mpu6050_kalman_filter_attitude_y_.setRmeasure(KalmanFilterParameter::r_measure);
}

void
IMU::calculateAttitudeMPU6050()
{
    /*
     *  Validate fast period.
     */
    if (PeriodParameter::fast <= 0)
    {
        Serial(LogLevel::error) << "Invalid fast period.";
        return;
    }

    /*
     *  Calculate the raw Y attitude (pitch) data using
     *  the populated linear accelerations in the member
     *  sensor data struct. Refer to the following materials
     *  to correctly convert the calculated data into the
     *  standard body reference frame.
     *
     *  Note that the attitudes (roll, pitch, and yaw) are
     *  angles between two pairs of acceleration vectors. Use
     *  atan2 function instead of atan for correct signedness.
     *
     *  Remember to perform the same calculation in the
     *  constructor for initialization but populate the
     *  member sensor data struct using the raw data (unfiltered)
     *  directly, since the Kalman filter had not been initialized
     *  at that point in the constructor.
     *
     *  Standard body reference frame:
     *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
     *
     *  Rotational right-hand rule:
     *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
     */
    // TODO LAB 5 YOUR CODE HERE.
    double accelX = mpu6050_data_.acceleration_x;
    double accelY = mpu6050_data_.acceleration_y;
    double accelZ = mpu6050_data_.acceleration_z;

    double attitude_x_raw = atan2(accelX, accelZ);
    double attitude_y_raw = atan2(accelY, accelZ);
    double attitude_z_raw = atan2(accelX, accelY);

; //TODO
    /*
     *  Filter the raw Y attitude data using the Kalman filter.
     */
    const double attitude_x_kalman_filter = mpu6050_kalman_filter_attitude_x_.getAngle(
            radiansToDegrees(attitude_x_raw), radiansToDegrees(mpu6050_data_.angular_velocity_x),
            PeriodParameter::fast);
    const double attitude_y_kalman_filter = mpu6050_kalman_filter_attitude_y_.getAngle(
            radiansToDegrees(attitude_y_raw), radiansToDegrees(mpu6050_data_.angular_velocity_y),
            PeriodParameter::fast);
    const double attitude_z_kalman_filter = mpu6050_kalman_filter_attitude_z_.getAngle(
            radiansToDegrees(attitude_z_raw), radiansToDegrees(mpu6050_data_.angular_velocity_z),
            PeriodParameter::fast);

    /*
     *  Convert the filtered Y attitude data back to radians
     *  using the degreesToRadians function, and populate
     *  the corresponding entry in the member sensor
     *  data struct.
     */
    // TODO LAB 5 YOUR CODE HERE.
    mpu6050_data_.attitude_x = degreesToRadians(attitude_x_kalman_filter);
    mpu6050_data_.attitude_y = degreesToRadians(attitude_y_kalman_filter);
    mpu6050_data_.attitude_z = degreesToRadians(attitude_z_kalman_filter);

}

void IMU::calculateAttitudeBMX160() {
    // Calculate raw attitude
    double attitude_x_raw = bmx160_data_.attitude_x;
    double attitude_y_raw = bmx160_data_.attitude_y;
    double attitude_z_raw = bmx160_data_.attitude_z;

    const double attitude_x_kalman_filter = bmx160_kalman_filter_attitude_x_.getAngle(
            radiansToDegrees(attitude_x_raw), radiansToDegrees(bmx160_data_.angular_velocity_x),
            PeriodParameter::fast);

    const double attitude_y_kalman_filter = bmx160_kalman_filter_attitude_y_.getAngle(
            radiansToDegrees(attitude_y_raw), radiansToDegrees(bmx160_data_.angular_velocity_y),
            PeriodParameter::fast);

    const double attitude_z_kalman_filter = bmx160_kalman_filter_attitude_z_.getAngle(
            radiansToDegrees(attitude_z_raw), radiansToDegrees(bmx160_data_.angular_velocity_z),
            PeriodParameter::fast);
    
    bmx160_data_.attitude_x = degreesToRadians(attitude_x_kalman_filter);
    bmx160_data_.attitude_y = degreesToRadians(attitude_y_kalman_filter);
    bmx160_data_.attitude_z = degreesToRadians(attitude_z_kalman_filter);
}
}
