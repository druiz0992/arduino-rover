
#include "sensor_LM393.h"
#include "sensor_HCSR04.h"
#include "sensor_MPU6050.h"
#include "controller_L298N.h"
#include "robot.h"
#include "two_wheel_robot.h"
#include "wheel_odometry.h"

Robot robot;

// Wheel encoder sensors
SensorLm393 lm393_right(LM393_DEFAULT_D0_PIN);
SensorLm393 lm393_left(LM393_LEFT_D0_PIN);
// IR Range sensor
SensorHcsr04 range(HCSR04_DEFAULT_TRIG_PIN, HCSR04_DEFAULT_ECHO_PIN);
// gyro
SensorMpu6050 gyro;


// Measurements
MeasurementLm393 lm393_sample_right;
MeasurementLm393 lm393_sample_left;
MeasurementHcsr04 range_sample;
//Measurement <Quaternion> q_sample;
MeasurementMpu6050 q_sample;

t_pin ln298n_pins_left[] = {L298N_DEFAULT_EN_PIN, L298N_DEFAULT_IN1_PIN, L298N_DEFAULT_IN2_PIN};
t_pin ln298n_pins_right[] = {LN298_RIGHT_EN_PIN, LN298_RIGHT_IN1_PIN, LN298_RIGHT_IN2_PIN};

ControllerL298N ln298n_left(ln298n_pins_left, false);
ControllerL298N ln298n_right(ln298n_pins_right, true);

// Odometry
WheelOdometry odometry(WHEEL_DIAMETER_CM, WHEEL_DISTANCE_CM, WHEEL_ENCODER_N_SECTIONS);


t_pose pose;

float dc = 0.1;
bool dc_up = true;

void setup() {
    Serial.begin(SERIAL_DEFAULT_SPEED);

    robot.installSensor(&lm393_right, &lm393_sample_right, LM393_DEFAULT_D0_PIN, CHANGE, LM393_RIGHT_SENSOR_IDX);
    robot.installSensor(&lm393_left, &lm393_sample_left, LM393_LEFT_D0_PIN, CHANGE, LM393_LEFT_SENSOR_IDX);
    robot.installSensor(&range, &range_sample, NO_ISR, NO_ISR, HCSR04_SENSOR_IDX);
    robot.installSensor(&gyro, &q_sample, NO_ISR, NO_ISR, MPU6050_SENSOR_IDX);

    robot.installController(&ln298n_right, LN298_RIGHT_CONTROLLER_IDX);
    robot.installController(&ln298n_left, LN298_LEFT_CONTROLLER_IDX);

    robot.initialize();

    /*
    lm393.initialize();
    range.initialize();
    gyro.initialize();
    */
}

void loop() {
    // Create a MeasurementUint16 object

    // Read the Measurement
    robot.readAndDispatchSensors();
    //lm393_right.read(&lm393_sample_right);
    //lm393_left.read(&lm393_sample_left);
    //range.read(&range_sample);
    //gyro.read(&q_sample);
    

    // Get the updated value
    //float value = robot.getMeasurement<float>(HCSR04_SENSOR_IDX);
    //Serial.println(value);
    /*
    int16_t value_right = robot.getMeasurement<int16_t>(LM393_RIGHT_SENSOR_IDX);
    int16_t value_left = robot.getMeasurement<int16_t>(LM393_LEFT_SENSOR_IDX);
    Serial.println("ticks");
    Serial.print(value_right);
    Serial.print(",");
    Serial.println(value_left);

    odometry.update(value_right, value_left);
    odometry.read(&pose);

    Serial.println("pose");
    Serial.print(pose.x);
    Serial.print(",");
    Serial.print(pose.y);
    Serial.print(",");
    Serial.println(pose.phi);
    */

    if (dc_up) {
       if (dc + 0.1 < L928N_MAX_DUTY_CYCLE) {
          dc+= 0.1;
       } else {
        dc_up = false;
        dc -= 0.1;
       }
    } else {
        if (dc - 0.1 > L928N_MIN_DUTY_CYCLE) {
            dc -= 0.1;
        } else {
            dc_up = true;
            dc += 0.1;
        }
    }
    ln298n_right.forward(dc);
    ln298n_left.forward(dc);
    //float updatedRangeValue = range_sample.getValue();
    //Serial.println(updatedRangeValue);
    /*
    uint16_t updatedLm393Value = lm393_sample.getValue();
    Quaternion q = q_sample.getValue();

    float ypr[YAW_PITCH_ROLL_DIMENSIONS];
    gyro.toYPR(&q, ypr);
    */



    //Serial.println(updatedLm393Value);
    /*
    Serial.print("Yaw: ");
    Serial.print(ypr[0]);
    Serial.print(", Pitch: ");
    Serial.print(ypr[1]);
    Serial.print(", Roll: ");
    Serial.println(ypr[2]);
    */

/*
    Serial.print("Quaternion - w: ");
    Serial.print(q.w);
    Serial.print(", x: ");
    Serial.print(q.x);
    Serial.print(", y: ");
    Serial.print(q.y);
    Serial.print(", z: ");
    Serial.println(q.z);
    */
   /*
    Serial.print(q.w);
    Serial.print(",");
    Serial.print(q.x);
    Serial.print(",");
    Serial.print(q.y);
    Serial.print(",");
    Serial.println(q.z);
    */

    delay(500);
}


#if 0



void setup() {
    Serial.begin(9600);

    gyro.initialize();
}

void loop() {
    float distance;
    range.read(&distance);
    Serial.println(distance);
    //float imu_sample[YAW_PITCH_ROLL_DIMENSIONS];
    /*
    Quaternion imu_sample;
    
    gyro.read(&imu_sample);
    Serial.print(imu_sample.w);
    Serial.print(",");
    Serial.print(imu_sample.x);
    Serial.print(",");
    Serial.print(imu_sample.y);
    Serial.print(",");
    Serial.println(imu_sample.z);

    Serial.println(imu_sample[0] * 180/M_PI);
    Serial.println(imu_sample[1] * 180/M_PI);
    Serial.println(imu_sample[2] * 180/M_PI);
    */

    delay(50);
}
#endif