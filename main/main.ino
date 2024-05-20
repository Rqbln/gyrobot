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
double kpPitch = 3.5, kiPitch = 1, kdPitch = 0.001;
double inputPitch, outputPitch, setPointPitch = 90.0;
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

// Low-pass filter for Z-axis accelerometer data
float zPrevious = 0;  // Previous filtered Z axis value
const float alphaZ = 0.1;  // Filter coefficient for Z axis

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

        // Apply low-pass filter to Z axis
        float zCurrent = az;
        float zFiltered = alphaZ * zCurrent + (1 - alphaZ) * zPrevious;
        zPrevious = zFiltered;

        // Calculate Pitch angle from accelerometer data using the filtered Z axis data
        inputPitch = ((zFiltered * 180.0 / 16384.0) + 180) * (setPointPitch * 2.0 / 360.0); // Scale az to degrees and adjust range to [0, setPointPitch * 2]
        double pitchError = setPointPitch - inputPitch;

        // If pitchError is less than 1, do not move the servos
        if (abs(pitchError) >= 1) {
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
            int commandLeft =
                    setPointPitch + (outputPitch >= 0 ? servoPower : -servoPower);
            int commandRight = setPointPitch - (outputPitch >= 0 ? servoPower : -servoPower);


            // Write commands to servos
            servoLeft.write(constrain(commandLeft, 0, 180));
            servoRight.write(constrain(commandRight, 0, 180));
        }

        // Debugging output
        Serial.print("Pitch Error: "); Serial.println(pitchError);

        previousTime = currentTime;
    }
}
