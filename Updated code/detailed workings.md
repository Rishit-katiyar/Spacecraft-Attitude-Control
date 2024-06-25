# Detailed Workings

```cpp
#include <Wire.h>  // Include the Wire library for I2C communication
#include <Adafruit_Sensor.h>  // Include Adafruit sensor library
#include <Adafruit_BNO055.h>  // Include BNO055 sensor library
```
These lines include the necessary libraries for I2C communication and the BNO055 sensor. The `Wire.h` library is used for I2C communication, while the `Adafruit_Sensor.h` and `Adafruit_BNO055.h` libraries are specific to the BNO055 sensor.

```cpp
#define GYRO_SENSITIVITY 0.07 // Gyroscope sensitivity in deg/s/LSB
#define ACCEL_SENSITIVITY 0.000061 // Accelerometer sensitivity in m/s^2/LSB
```
These lines define constants for the sensitivity of the gyroscope and accelerometer. The values convert raw sensor data to meaningful units.

```cpp
#define REACTION_WHEEL_PIN 9  // Pin for reaction wheel control
#define THRUSTER_PIN 10       // Pin for thruster control
#define MAGNETORQUER_PIN 11   // Pin for magnetorquer control
```
These lines define the pins connected to the actuators. The `REACTION_WHEEL_PIN`, `THRUSTER_PIN`, and `MAGNETORQUER_PIN` are set to pins 9, 10, and 11, respectively.

```cpp
#define ROLL_KP 1.0
#define ROLL_KI 0.0
#define ROLL_KD 0.0
#define PITCH_KP 1.0
#define PITCH_KI 0.0
#define PITCH_KD 0.0
#define YAW_KP 1.0
#define YAW_KI 0.0
#define YAW_KD 0.0
```
These lines define the PID constants (Kp, Ki, Kd) for the roll, pitch, and yaw axes. These constants are used in the PID controller to calculate the control output.

```cpp
const float target_roll = 0.0;
const float target_pitch = 0.0;
const float target_yaw = 0.0;
```
These lines set the target angles for the roll, pitch, and yaw axes. The spacecraft aims to maintain these angles.

```cpp
float gyro_x, gyro_y, gyro_z; // Gyroscope readings in deg/s
float accel_x, accel_y, accel_z; // Accelerometer readings in m/s^2
```
These variables store the sensor data from the gyroscope and accelerometer.

```cpp
float roll_error, roll_integral = 0.0, roll_derivative = 0.0;
float pitch_error, pitch_integral = 0.0, pitch_derivative = 0.0;
float yaw_error, yaw_integral = 0.0, yaw_derivative = 0.0;
float roll_output, pitch_output, yaw_output;
unsigned long last_time = 0;
```
These variables store the error, integral, derivative, and output for the PID controller of each axis. The `last_time` variable is used to calculate the time difference between PID updates.

```cpp
Adafruit_BNO055 bno = Adafruit_BNO055();
```
This line creates an instance of the BNO055 sensor.

```cpp
float pid_controller(float error, float &integral, float &prev_error, float Kp, float Ki, float Kd) {
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0; // Calculate time difference since last iteration
    last_time = current_time;
    
    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    
    float output = Kp * error + Ki * integral + Kd * derivative;
    
    prev_error = error;
    
    return output;
}
```
This function calculates the PID control output. It takes the current error, integral, previous error, and PID constants as inputs. It calculates the integral and derivative terms and uses them to compute the PID output. The integral and previous error are passed by reference, allowing them to be updated for the next iteration.

```cpp
void setup() {
    Serial.begin(9600);  // Start serial communication at 9600 baud
    Wire.begin();  // Initialize I2C communication
    
    if (!bno.begin()) {
        Serial.print("No BNO055 detected. Check your wiring or I2C ADDR!");
        while (1);
    }
    
    pinMode(REACTION_WHEEL_PIN, OUTPUT);
    pinMode(THRUSTER_PIN, OUTPUT);
    pinMode(MAGNETORQUER_PIN, OUTPUT);
}
```
The `setup` function initializes the serial communication, I2C communication, and the BNO055 sensor. It also sets the actuator pins as outputs. If the BNO055 sensor is not detected, it prints an error message and stops execution.

```cpp
void loop() {
    sensors_event_t event;
    bno.getEvent(&event);
    
    gyro_x = event.gyro.x * GYRO_SENSITIVITY;
    gyro_y = event.gyro.y * GYRO_SENSITIVITY;
    gyro_z = event.gyro.z * GYRO_SENSITIVITY;
    
    accel_x = event.acceleration.x * ACCEL_SENSITIVITY;
    accel_y = event.acceleration.y * ACCEL_SENSITIVITY;
    accel_z = event.acceleration.z * ACCEL_SENSITIVITY;
    
    roll_error = target_roll - atan2(-accel_y, accel_z) * 180 / PI;
    pitch_error = target_pitch - atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / PI;
    yaw_error = target_yaw - gyro_z;
    
    roll_output = pid_controller(roll_error, roll_integral, roll_derivative, ROLL_KP, ROLL_KI, ROLL_KD);
    pitch_output = pid_controller(pitch_error, pitch_integral, pitch_derivative, PITCH_KP, PITCH_KI, PITCH_KD);
    yaw_output = pid_controller(yaw_error, yaw_integral, yaw_derivative, YAW_KP, YAW_KI, YAW_KD);
    
    analogWrite(REACTION_WHEEL_PIN, constrain(roll_output, 0, 255));
    analogWrite(THRUSTER_PIN, constrain(pitch_output, 0, 255));
    analogWrite(MAGNETORQUER_PIN, constrain(yaw_output, 0, 255));
    
    Serial.print("Roll Error: "); Serial.print(roll_error); Serial.print("\t");
    Serial.print("Pitch Error: "); Serial.print(pitch_error); Serial.print("\t");
    Serial.print("Yaw Error: "); Serial.println(yaw_error);
    
    Serial.print("Roll Output: "); Serial.print(roll_output); Serial.print("\t");
    Serial.print("Pitch Output: "); Serial.print(pitch_output); Serial.print("\t");
    Serial.print("Yaw Output: "); Serial.println(yaw_output);
    
    delay(10); // Delay to control loop rate
}
```
The `loop` function is executed repeatedly. Here’s what happens step by step:

1. **Reading Sensor Data**:
    - The `bno.getEvent(&event)` function reads the sensor data.
    - Gyroscope readings (`event.gyro.x`, `event.gyro.y`, `event.gyro.z`) are converted to degrees per second using `GYRO_SENSITIVITY`.
    - Accelerometer readings (`event.acceleration.x`, `event.acceleration.y`, `event.acceleration.z`) are converted to meters per second squared using `ACCEL_SENSITIVITY`.

2. **Calculating Errors**:
    - `roll_error` is calculated using the arctangent of `-accel_y / accel_z`, converting the result to degrees.
    - `pitch_error` is calculated using the arctangent of `accel_x / sqrt(accel_y^2 + accel_z^2)`, converting the result to degrees.
    - `yaw_error` is calculated directly from the gyroscope's z-axis reading.

3. **PID Control**:
    - The `pid_controller` function is called for each axis (roll, pitch, yaw) to calculate the control outputs (`roll_output`, `pitch_output`, `yaw_output`).

4. **Actuating the Spacecraft**:
    - The control outputs are constrained to valid PWM ranges (0-255) and applied to the actuators using `analogWrite`.

5. **Debugging Output**:
    - Sensor errors and PID outputs are printed to the serial monitor for debugging purposes.

6. **Loop Delay**:
    - A delay of 10 milliseconds is added to control the loop rate, making the loop run approximately 100 times per second.

This code allows the spacecraft to maintain its orientation by continuously reading sensor data, calculating errors, applying PID control, and actuating the relevant hardware.
