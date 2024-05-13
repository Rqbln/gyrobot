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
double kpPitch = 2, kiPitch =1 , kdPitch =0.01 ;
double inputPitch, outputPitch, setPointPitch = 90.0;
double lastErrorPitch = 0;

// PID parameters for Roll
double kpRoll = 0, kiRoll = 0, kdRoll = 0;
double inputRoll, outputRoll, setPointRoll = 0.0;
double lastErrorRoll = 0;

// Timing
unsigned long currentTime, previousTime;
double sampleTime = 23; // Sample time in milliseconds

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
        outputPitch = kpPitch * pitchError + kiPitch * pitchError * (sampleTime / 1000) + kdPitch * (pitchError - lastErrorPitch) / (sampleTime / 1000);
        lastErrorPitch = pitchError;

        // Calculate Roll angle from gyroscope data
        inputRoll = gx / 131.0; // Convert gyro data to degrees/s
        double rollError = setPointRoll - inputRoll;
        outputRoll = kpRoll * rollError + kiRoll * rollError * (sampleTime / 1000) + kdRoll * (rollError - lastErrorRoll) / (sampleTime / 1000);
        lastErrorRoll = rollError;

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