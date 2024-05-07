#include "Wire.h"
#include <Arduino.h>
#include <Servo.h>

// Adresse I2C du MPU6050
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Variables pour les angles
double angleX; // Angle en degrés sur l'axe X

Servo servoLeft;  // Servomoteur gauche
Servo servoRight; // Servomoteur droit

// Paramètres du contrôleur PID
double setPoint = 87.0; // Angle cible de tangage à 90 degrés
double kp = 1;          // Coefficient proportionnel
double ki = 0;          // Coefficient intégral
double kd = 0;          // Coefficient dérivé

double error, integral = 0, derivative, lastError = 0;
double output;

void setup() {
    Wire.begin(); // Commence la communication I2C
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // Réveille le MPU6050
    Wire.write(0);
    Wire.endTransmission(true);

    Serial.begin(9600); // Commence la communication série pour le debug

    servoLeft.attach(10);  // Attache le servomoteur gauche au pin D10
    servoRight.attach(11); // Attache le servomoteur droit au pin D11
}

void loop() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // Demande les données à partir du registre 0x3B
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // Lit 14 octets de données

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();

    // Calcule l'angle de tangage à partir des accéléromètres
    angleX = atan2(AcY, AcZ) * 180.0 / PI;

    // Calcul du PID
    error = setPoint - angleX;
    integral += error * 0.001;
    derivative = (error - lastError) / 0.001;
    output = kp * error + ki * integral + kd * derivative;
    lastError = error;

    // Commande PID ajustée pour les servomoteurs
    int commandLeft = 90 + output;  // Commande ajustée pour le servo gauche
    int commandRight = 90 - output; // Commande ajustée pour le servo droit

    // Ajuste les commandes pour rester dans les limites des servomoteurs
    commandLeft = constrain(commandLeft, 0, 180);
    commandRight = constrain(commandRight, 0, 180);

    servoLeft.write(commandLeft);
    servoRight.write(commandRight); // Maintenant correctement inversé pour l'orientation

    // Affichage des informations pour le débogage
    Serial.print("Angle X: "); Serial.println(angleX);
    Serial.print("Command Left: "); Serial.print(commandLeft);
    Serial.print(" Command Right: "); Serial.println(commandRight);

    delay(1); // Délai pour la boucle de contrôle
}
