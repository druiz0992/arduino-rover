
#include "sensor_LM393.h"
#include "sensor_HCSR04.h"
#include "sensor_MPU6050.h"
#include "controller_L298N.h"
#include "robot.h"
#include "two_wheel_robot.h"
#include "wheel_odometry.h"

#define DEBUG_ENABLE false

Robot robot(DEBUG_ENABLE);

// Wheel encoder sensors
SensorLm393 lm393_right(LM393_DEFAULT_D0_PIN);
SensorLm393 lm393_left(LM393_LEFT_D0_PIN);
SensorLm393Combined lm393(lm393_left, lm393_right);

// IR Range sensor
SensorHcsr04 range(HCSR04_DEFAULT_TRIG_PIN, HCSR04_DEFAULT_ECHO_PIN);
// gyro
SensorMpu6050 gyro;

// Measurements
MeasurementLm393 lm393_sample_right;
MeasurementLm393 lm393_sample_left;
MeasurementLm393Combined lm393_samples;

MeasurementHcsr04 range_sample;
// Measurement <Quaternion> q_sample;
MeasurementMpu6050 q_sample;
MeasurementMpu6050Accel accel_sample;

// Motor controller
t_pin l298n_pins_left[] = {L298N_DEFAULT_EN_PIN, L298N_DEFAULT_IN1_PIN, L298N_DEFAULT_IN2_PIN};
t_pin l298n_pins_right[] = {L298N_RIGHT_EN_PIN, L298N_RIGHT_IN1_PIN, L298N_RIGHT_IN2_PIN};

ControllerL298N l298n_left(l298n_pins_left, false);
ControllerL298N l298n_right(l298n_pins_right, true);
CombinedControllerL298N wheels(&l298n_left, &l298n_right);
MeasurementL298N wheel_sample;

// Odometry
WheelOdometry odometry(WHEEL_DIAMETER_CM, WHEEL_DISTANCE_CM, WHEEL_ENCODER_N_SECTIONS, &wheel_sample, &lm393_samples);
MeasurementPose odometry_sample;

void setup()
{
    Serial1.begin(SERIAL_DEFAULT_SPEED);

    // wheel encoder
    robot.installSensor(&lm393_right, &lm393_sample_right, LM393_DEFAULT_D0_PIN, RISING, LM393_RIGHT_SENSOR_IDX, nullptr);
    robot.installSensor(&lm393_left, &lm393_sample_left, LM393_LEFT_D0_PIN, RISING, LM393_LEFT_SENSOR_IDX, nullptr);
    robot.installSensor(&lm393, &lm393_samples, NO_ISR, NO_ISR, LM393_SENSOR_IDX, LM393_SENSOR_CHANNEL);

    // range sensor
    robot.installSensor(&range, &range_sample, NO_ISR, NO_ISR, HCSR04_SENSOR_IDX, HCSR04_SENSOR_CHANNEL);
    // gyro
    robot.installSensor(&gyro, &q_sample, NO_ISR, NO_ISR, MPU6050_SENSOR_IDX, MPU6050_SENSOR_CHANNEL);
    // accel 
    robot.installSensor(&gyro, &accel_sample, NO_ISR, NO_ISR, MPU6050_ACCEL_SENSOR_IDX, MPU6050_ACCEL_SENSOR_CHANNEL);

    // motor controllers
    robot.installController(&wheels, &wheel_sample, L298N_CONTROLLER_IDX, L298N_CONTROLLER_CHANNEL);

    // processors
    robot.installProcessor(&odometry, &odometry_sample, WHEEL_ODOMETRY_PROCESSOR_IDX, WHEEL_ODOMETRY_PROCESSOR_CHANNEL);


    robot.initialize();

    lm393_left.initialize();
    lm393_right.initialize();
    range.initialize();
    gyro.initialize();
}

void loop()
{
    robot.readCommand();
    robot.readAndDispatchSensors();
    robot.processMeasurements();

    delay(300);
}