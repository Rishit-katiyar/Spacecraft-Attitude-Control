#include <Wire.h>  // Include Wire library for I2C communication
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>  // Include BNO055 sensor library

#define GYRO_SENSITIVITY 0.07 // Gyroscope sensitivity in deg/s/LSB
#define ACCEL_SENSITIVITY 0.000061 // Accelerometer sensitivity in m/s^2/LSB

// Mock sensor data
float mock_gyro_x = 0.0, mock_gyro_y = 0.0, mock_gyro_z = 0.0;
float mock_accel_x = 0.0, mock_accel_y = 0.0, mock_accel_z = 9.8; // Assuming 1g for simplicity

// Function prototypes
void mockSensorData();

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    // No need to initialize actuators in test code
}

void loop() {
    mockSensorData(); // Mock sensor data
    
    // Mock errors for PID controllers
    float mock_roll_error = 0.0; // Assuming target roll is 0
    float mock_pitch_error = 0.0; // Assuming target pitch is 0
    float mock_yaw_error = 0.0; // Assuming target yaw is 0
    
    // Mock PID outputs
    float mock_roll_output = 0.0;
    float mock_pitch_output = 0.0;
    float mock_yaw_output = 0.0;
    
    // Print sensor data and PID outputs
    Serial.print("Roll Error: "); Serial.print(mock_roll_error); Serial.print("\t");
    Serial.print("Pitch Error: "); Serial.print(mock_pitch_error); Serial.print("\t");
    Serial.print("Yaw Error: "); Serial.println(mock_yaw_error);
    
    Serial.print("Roll Output: "); Serial.print(mock_roll_output); Serial.print("\t");
    Serial.print("Pitch Output: "); Serial.print(mock_pitch_output); Serial.print("\t");
    Serial.print("Yaw Output: "); Serial.println(mock_yaw_output);
    
    delay(1000); // Simulate loop rate of 1Hz
}

// Function to mock sensor data
void mockSensorData() {
    // Mock gyro data
    mock_gyro_x += 0.1; // Example increment for mock data
    mock_gyro_y -= 0.1;
    mock_gyro_z += 0.05;
    
    // Mock accel data (assuming constant acceleration due to gravity in z-axis)
    mock_accel_x += 0.01; // Example increment for mock data
    mock_accel_y -= 0.01;
    
    // Print mock sensor data (for testing purposes)
    Serial.print("Mock Gyro X: "); Serial.print(mock_gyro_x); Serial.print("\t");
    Serial.print("Mock Gyro Y: "); Serial.print(mock_gyro_y); Serial.print("\t");
    Serial.print("Mock Gyro Z: "); Serial.print(mock_gyro_z); Serial.print("\t");
    Serial.print("Mock Accel X: "); Serial.print(mock_accel_x); Serial.print("\t");
    Serial.print("Mock Accel Y: "); Serial.println(mock_accel_y);
}
