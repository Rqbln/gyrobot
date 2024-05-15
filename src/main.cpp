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
double kpPitch = 0.8, kiPitch = 4, kdPitch = 0.0005;
double inputPitch, outputPitch, setPointPitch = 91.0;
double lastErrorPitch = 0;
double integralPitch = 0;
double lastInputPitch = 0;

// Timing
unsigned long currentTime, previousTime;
double sampleTime = 10; // Sample time in milliseconds

// Low-pass filter constants
double tau = 0.05; // Time constant for low-pass filter
double alpha = tau / (tau + sampleTime / 1000.0);

// Low-pass filtered derivative term
double filteredDPitch = 0;

void setup() {
    Wire.begin();
    Serial.begin(9600);
    mpu.initialize();

    servoLeft.attach(10);
    servoRight.attach(11);

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

        // Scale outputPitch to determine servo power (0-180 range)
        int servoPower = map(abs(outputPitch), 0, 180, 0, 90);

        // Reduce correction variablely as the gyropode approaches setPointPitch for angles > setPointPitch
        double correctionFactor = 1.0;
        if (inputPitch > setPointPitch) {
            if (abs(pitchError) < 10) {
                correctionFactor = 0.5 + 0.05 * abs(pitchError); // Adjust this function as needed
            } else if (abs(pitchError) < 20) {
                correctionFactor = 0.75 + 0.0125 * abs(pitchError);
            }
            servoPower = servoPower * correctionFactor;
        }

        // Adjust servo commands based on the scaled power
        int commandLeft = setPointPitch - (outputPitch >= 0 ? servoPower : -servoPower);
        int commandRight = setPointPitch + (outputPitch >= 0 ? servoPower : -servoPower);

        // Write commands to servos
        servoLeft.write(constrain(commandLeft, 0, 180));
        servoRight.write(constrain(commandRight, 0, 180));

        // Debugging output
        Serial.print("Pitch: "); Serial.println(inputPitch);
        Serial.print("Pitch Error: "); Serial.println(pitchError);
        Serial.print("Left Command: "); Serial.println(commandLeft);
        Serial.print("Right Command: "); Serial.println(commandRight);

        previousTime = currentTime;
    }
}
