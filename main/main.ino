#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// MPU6050 object
MPU6050 mpu;

// Servo objects
Servo servoLeft;
Servo servoRight;

// PID parameters for Pitch
double kpPitch = 0.8, kiPitch = 10, kdPitch = 0.0005;
double inputPitch, outputPitch, setPointPitch = 90.0;
double lastErrorPitch = 0;
double integralPitch = 0;
double lastInputPitch = 0;

// PID parameters for Roll
double kpRoll = 0, kiRoll = 0, kdRoll = 0;
double inputRoll, outputRoll, setPointRoll = 0.0;
double lastErrorRoll = 0;
double integralRoll = 0;
double lastInputRoll = 0;

// Timing
unsigned long currentTime, previousTime;
double sampleTime = 10; // Sample time in milliseconds

// Low-pass filter constants
double tau = 0.5; // Time constant for low-pass filter
double alpha = tau / (tau + sampleTime / 1000.0);

// Low-pass filtered derivative terms
double filteredDPitch = 0;
double filteredDRoll = 0;

void setup() {
    Wire.begin();
    Serial.begin(9600);
    mpu.initialize();

    servoLeft.attach(11);
    servoRight.attach(10);

    previousTime = millis();
}

void loop() {
    currentTime = millis();
    if (currentTime - previousTime >= sampleTime) {
        // Read motion sensor data
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // Calculate Pitch angle from accelerometer data
        inputPitch = atan2(ay, az) * 180.0 / PI;
        double pitchError = setPointPitch - inputPitch;
        
        // Proportional term
        double pTermPitch = kpPitch * pitchError;
        
        // Integral term with anti-windup
        integralPitch += kiPitch * pitchError * (sampleTime / 1000.0);
        integralPitch = constrain(integralPitch, -100, 100); // Clamp integral term
        
        // Derivative term with low-pass filter
        double dTermPitch = kdPitch * (inputPitch - lastInputPitch) / (sampleTime / 1000.0);
        filteredDPitch = alpha * dTermPitch + (1 - alpha) * filteredDPitch;

        // PID output for Pitch
        outputPitch = pTermPitch + integralPitch - filteredDPitch;
        lastInputPitch = inputPitch;

        // Calculate Roll angle from gyroscope data
        inputRoll = gx / 131.0; // Convert gyro data to degrees/s
        double rollError = setPointRoll - inputRoll;
        
        // Proportional term
        double pTermRoll = kpRoll * rollError;
        
        // Integral term with anti-windup
        integralRoll += kiRoll * rollError * (sampleTime / 1000.0);
        integralRoll = constrain(integralRoll, -100, 100); // Clamp integral term
        
        // Derivative term with low-pass filter
        double dTermRoll = kdRoll * (inputRoll - lastInputRoll) / (sampleTime / 1000.0);
        filteredDRoll = alpha * dTermRoll + (1 - alpha) * filteredDRoll;

        // PID output for Roll
        outputRoll = pTermRoll + integralRoll - filteredDRoll;
        lastInputRoll = inputRoll;

        // Combine outputs to control servos
        int commandLeft = 90 - outputPitch - outputRoll;
        int commandRight = 90 + outputPitch + outputRoll;

        servoLeft.write(constrain(commandLeft, 0, 180));
        servoRight.write(constrain(commandRight, 0, 180));

        // Debugging output
        Serial.print("Pitch: "); Serial.println(inputPitch);
        Serial.print("Roll: "); Serial.println(inputRoll);
        Serial.print("Left Command: "); Serial.println(commandLeft);
        Serial.print("Right Command: "); Serial.println(commandRight);

        previousTime = currentTime;
    }
}
