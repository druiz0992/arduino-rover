
#ifndef __SENSOR_MPU6050_H__
#define __SENSOR_MPU6050_H__

#include "sensor.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define DEFAULT_SDA_PIN 20
#define DEFAULT_SCL_PIN 21

#define STATUS_OK 0
#define STATUS_MEMORY_LOAD_FAILED 1
#define STATUS_DMP_CONFIGURATION_FAILED 2
#define STATUS_CONNECTION_FAILED 3
#define STATUS_ERROR 4
#define STATUS_NOT_INITIALIZED 5

#define I2C_CLOCK_FREQUENCY 400000

#define ACCEL_SENSITIVITY_FACTOR 16384

// Amount of readings used to average. Make it larger for more precision
#define DEFAULT_BUFFER_SIZE 1000
#define DEFAULT_NUMBER_DISCARDED_MEASUREMENTS 100
// Accelerometer error allowed. Lower for more precision. Don't make this 0
#define DEFAULT_ACCEL_ERROR 8
// Gyro error allowed. Lower for more precision. Don't mke this 0
#define DEFAULT_GYRO_ERROR 4

#define DEFAULT_CALIBRATION_RETRIES 10

#define IRQ_FIFO_OVERFLOW_CODE 0x10
#define FIFO_MAX_SIZE 1024
#define IRQ_STATUS_OK 0x2
#define BUFFER_SIZE 64

#define YAW_PITCH_ROLL_DIMENSIONS 3

#define BUFFER_SIZE DEFAULT_BUFFER_SIZE
#define ACCEL_ERROR DEFAULT_ACCEL_ERROR
#define GYRO_ERROR DEFAULT_GYRO_ERROR
#define CALIBRATION_RETRIES DEFAULT_CALIBRATION_RETRIES

#define GYRO_CALIBRATION_FACTOR 4
#define ACCEL_CALIBRATION_FACTOR 7.8

#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE

#define SENSOR_IMU_DEBUG

#define DEFAULT_X_ACCEL_OFFSET 642
#define DEFAULT_Y_ACCEL_OFFSET 3412
#define DEFAULT_Z_ACCEL_OFFSET 691
#define DEFAULT_X_GYRO_OFFSET 89
#define DEFAULT_Y_GYRO_OFFSET -53
#define DEFAULT_Z_GYRO_OFFSET 41

#define ENABLE_CALIBRATION false

#define MPU6050_MAX_MEASUREMENT_LEN 10
#define MPU6050_MEASUREMENT_MIN_WIDTH 4
#define MPU6050_MEASUREMENT_PRECISION 2

typedef struct
{
    int ax;
    int ay;
    int az;

    int gx;
    int gy;
    int gz;

} t_calibration_data;

class MeasurementMpu6050 : public Measurement<Quaternion>
{
public:
    void toString(char *str) const override
    {
        char _value_w_str[MPU6050_MAX_MEASUREMENT_LEN];
        char _value_x_str[MPU6050_MAX_MEASUREMENT_LEN];
        char _value_y_str[MPU6050_MAX_MEASUREMENT_LEN];
        char _value_z_str[MPU6050_MAX_MEASUREMENT_LEN];
        dtostrf(_value.w, MPU6050_MEASUREMENT_MIN_WIDTH, MPU6050_MEASUREMENT_PRECISION, _value_w_str);
        dtostrf(_value.x, MPU6050_MEASUREMENT_MIN_WIDTH, MPU6050_MEASUREMENT_PRECISION, _value_x_str);
        dtostrf(_value.y, MPU6050_MEASUREMENT_MIN_WIDTH, MPU6050_MEASUREMENT_PRECISION, _value_y_str);
        dtostrf(_value.z, MPU6050_MEASUREMENT_MIN_WIDTH, MPU6050_MEASUREMENT_PRECISION, _value_z_str);
        sprintf(str, "%s,%s,%s,%s", _value_w_str, _value_x_str, _value_y_str, _value_z_str);
    }
};

class SensorMpu6050 : public Sensor
{
public:
    SensorMpu6050();

    void initialize() override;
    void calibrate() override;
    void read(MeasurementBase *sample) override;
    void isr() override;
    void toYPR(Quaternion *from, float *to);

private:
    int _sda_pin;
    int _scl_pin;

    uint8_t _status;

    t_calibration_data _calibration;

    MPU6050 _mpu6050;
    volatile float _ypr_sample[YAW_PITCH_ROLL_DIMENSIONS];

    void _averageReadings(t_calibration_data *);
};

#endif /* Sensor_MPU6050 */