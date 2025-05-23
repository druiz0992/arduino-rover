#ifndef __TWO_WHEEL_ROBOT_H__
#define __TWO_WHEEL_ROBOT_H__

#define SERIAL_DEFAULT_SPEED 115200
#define LM393_LEFT_D0_PIN 3

#define L298N_RIGHT_EN_PIN 10
#define L298N_RIGHT_IN1_PIN 7
#define L298N_RIGHT_IN2_PIN 8

#define L298N_CONTROLLER_IDX 0

#define LM393_RIGHT_SENSOR_IDX 0
#define LM393_LEFT_SENSOR_IDX 1
#define LM393_SENSOR_IDX 2
#define HCSR04_SENSOR_IDX 3
#define MPU6050_SENSOR_IDX 4
#define MPU6050_ACCEL_SENSOR_IDX 5

#define LM393_RIGHT_SENSOR_CHANNEL "lm393_right"
#define LM393_LEFT_SENSOR_CHANNEL "lm393_left"
#define LM393_SENSOR_CHANNEL "lm393"
#define HCSR04_SENSOR_CHANNEL "hcsr04"
#define MPU6050_SENSOR_CHANNEL "mpu6050"
#define MPU6050_ACCEL_SENSOR_CHANNEL "mpu6050_accel"
#define L298N_CONTROLLER_CHANNEL "l298n"

#define WHEEL_ODOMETRY_PROCESSOR_IDX 0
#define WHEEL_ODOMETRY_PROCESSOR_CHANNEL "odometry"

#define WHEEL_DIAMETER_CM 6.6
#define WHEEL_DISTANCE_CM 14.0
#define WHEEL_ENCODER_N_SECTIONS 20.0

#endif