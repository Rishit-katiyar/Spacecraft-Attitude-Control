#include <Wire.h>  // Include Wire library for I2C communication
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>  // Include BNO055 sensor library

#define GYRO_SENSITIVITY 0.07 // Gyroscope sensitivity in deg/s/LSB
#define ACCEL_SENSITIVITY 0.000061 // Accelerometer sensitivity in m/s^2/LSB

// Pin definitions for actuators
#define REACTION_WHEEL_PIN 9  // Example pin for reaction wheel control
#define THRUSTER_PIN 10        // Example pin for thruster control
#define MAGNETORQUER_PIN 11    // Example pin for magnetorquer control

// Define PID constants for roll, pitch, and yaw axes
#define ROLL_KP 1.0
#define ROLL_KI 0.0
#define ROLL_KD 0.0
#define PITCH_KP 1.0
#define PITCH_KI 0.0
#define PITCH_KD 0.0
#define YAW_KP 1.0
#define YAW_KI 0.0
#define YAW_KD 0.0

// Target angles for roll, pitch, and yaw axes (in degrees)
float target_roll = 0.0;
float target_pitch = 0.0;
float target_yaw = 0.0;

// Variables for storing sensor data
float gyro_x, gyro_y, gyro_z; // Gyroscope readings in deg/s
float accel_x, accel_y, accel_z; // Accelerometer readings in m/s^2

// Variables for PID controller
float roll_error, roll_integral, roll_derivative;
float pitch_error, pitch_integral, pitch_derivative;
float yaw_error, yaw_integral, yaw_derivative;
float roll_output, pitch_output, yaw_output;
unsigned long last_time = 0;

// Initialize BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055();

// PID controller function for roll axis
float pid_controller(float error, float *integral, float *prev_error, float Kp, float Ki, float Kd) {
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0; // Calculate time difference since last iteration
    last_time = current_time;
    
    // Calculate integral and derivative terms
    *integral += error * dt;
    float derivative = (error - *prev_error) / dt;
    
    // Calculate PID output
    float output = Kp * error + Ki * (*integral) + Kd * derivative;
    
    // Update previous error for next iteration
    *prev_error = error;
    
    return output;
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    bno.begin();
    
    // Initialize actuators
    pinMode(REACTION_WHEEL_PIN, OUTPUT);
    pinMode(THRUSTER_PIN, OUTPUT);
    pinMode(MAGNETORQUER_PIN, OUTPUT);
}

void loop() {
    // Read sensor data
    sensors_event_t event;
    bno.getEvent(&event);
    
    gyro_x = event.gyro.x * GYRO_SENSITIVITY;
    gyro_y = event.gyro.y * GYRO_SENSITIVITY;
    gyro_z = event.gyro.z * GYRO_SENSITIVITY;
    
    accel_x = event.acceleration.x * ACCEL_SENSITIVITY;
    accel_y = event.acceleration.y * ACCEL_SENSITIVITY;
    accel_z = event.acceleration.z * ACCEL_SENSITIVITY;
    
    // Calculate errors for PID controllers
    roll_error = target_roll - atan2(-accel_y, accel_z) * 180 / PI;
    pitch_error = target_pitch - atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / PI;
    yaw_error = target_yaw - gyro_z;
    
    // Apply PID control
    roll_output = pid_controller(roll_error, &roll_integral, &roll_derivative, ROLL_KP, ROLL_KI, ROLL_KD);
    pitch_output = pid_controller(pitch_error, &pitch_integral, &pitch_derivative, PITCH_KP, PITCH_KI, PITCH_KD);
    yaw_output = pid_controller(yaw_error, &yaw_integral, &yaw_derivative, YAW_KP, YAW_KI, YAW_KD);
    
    // Actuate the spacecraft using the PID outputs
    // Example: apply the outputs to control actuators
    analogWrite(REACTION_WHEEL_PIN, roll_output);
    analogWrite(THRUSTER_PIN, pitch_output);
    analogWrite(MAGNETORQUER_PIN, yaw_output);
    
    // Print sensor data and PID outputs
    Serial.print("Roll: "); Serial.print(roll_error); Serial.print("\t");
    Serial.print("Pitch: "); Serial.print(pitch_error); Serial.print("\t");
    Serial.print("Yaw: "); Serial.println(yaw_error);
    
    Serial.print("Roll Output: "); Serial.print(roll_output); Serial.print("\t");
    Serial.print("Pitch Output: "); Serial.print(pitch_output); Serial.print("\t");
    Serial.print("Yaw Output: "); Serial.println(yaw_output);
    
    delay(10); // Delay to control loop rate
}
