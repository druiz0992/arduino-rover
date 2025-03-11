
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
// IR Range sensor
SensorHcsr04 range(HCSR04_DEFAULT_TRIG_PIN, HCSR04_DEFAULT_ECHO_PIN);
// gyro
SensorMpu6050 gyro;

// Measurements
MeasurementLm393Right lm393_sample_right;
MeasurementLm393Left lm393_sample_left;
MeasurementHcsr04 range_sample;
// Measurement <Quaternion> q_sample;
MeasurementMpu6050 q_sample;

t_pin ln298n_pins_left[] = {L298N_DEFAULT_EN_PIN, L298N_DEFAULT_IN1_PIN, L298N_DEFAULT_IN2_PIN};
t_pin ln298n_pins_right[] = {LN298_RIGHT_EN_PIN, LN298_RIGHT_IN1_PIN, LN298_RIGHT_IN2_PIN};

ControllerL298N ln298n_left(ln298n_pins_left, false);
ControllerL298N ln298n_right(ln298n_pins_right, true);
CombinedControllerL298N wheels(&ln298n_left, &ln298n_right);

// Odometry
WheelOdometry odometry(WHEEL_DIAMETER_CM, WHEEL_DISTANCE_CM, WHEEL_ENCODER_N_SECTIONS);

t_pose pose;

float dc = 0.1;
bool dc_up = true;
bool forward = true;

void setup()
{
    Serial1.begin(SERIAL_DEFAULT_SPEED);

    // wheel encoder
    robot.installSensor(&lm393_right, &lm393_sample_right, LM393_DEFAULT_D0_PIN, CHANGE, LM393_RIGHT_SENSOR_IDX, LM393_RIGHT_SENSOR_CHANNEL);
    robot.installSensor(&lm393_left, &lm393_sample_left, LM393_LEFT_D0_PIN, CHANGE, LM393_LEFT_SENSOR_IDX, LM393_LEFT_SENSOR_CHANNEL);
    // range sensor
    robot.installSensor(&range, &range_sample, NO_ISR, NO_ISR, HCSR04_SENSOR_IDX, HCSR04_SENSOR_CHANNEL);
    // gyro
    robot.installSensor(&gyro, &q_sample, NO_ISR, NO_ISR, MPU6050_SENSOR_IDX, MPU6050_SENSOR_CHANNEL);

    // motor controllers
    robot.installController(&wheels, LN298_CONTROLLER_IDX, LN298_CONTROLLER_CHANNEL);

    robot.initialize();

    lm393_left.initialize();
    lm393_right.initialize();
    range.initialize();
    gyro.initialize();
}

void loop()
{
    robot.readAndDispatchSensors();
    robot.readCommand();

    delay(300);
}