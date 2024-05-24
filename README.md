# Gyrobot Project

**Titre du projet :** Gyrobot - Robot auto-équilibrant

**Auteurs :** Laouïg ELEOUET, Robin QUERIAUX

**Enseignant :** M. LE GALL

**Date :** Paris, 20/05/2024

**Description du projet :**
Développement d'un robot auto-équilibrant à deux roues utilisant le capteur MPU6050 pour mesurer l'inclinaison et la vitesse. Le robot se stabilise verticalement, maintient une direction et conserve une vitesse constante. Les défis incluent l'intégration des composants électroniques, la programmation des contrôleurs PID et l'adaptation aux variations des composants.

## Introduction

Le projet Gyrobot vise à développer un gyropode auto-équilibrant à deux roues utilisant l'accéléromètre et le gyroscope MPU6050 pour mesurer l'inclinaison et la vitesse, offrant une stabilisation verticale, une navigation précise et une gestion de la vitesse constante. Ce gyropode est conçu dans un contexte pédagogique pour appliquer des connaissances en systèmes embarqués et contrôle PID.

## Structure du Répertoire

- **.idea**: Contient les configurations spécifiques à l'IDE utilisé.
- **include**: Dossiers pour les fichiers d'en-tête.
- **lib**: Bibliothèques tierces nécessaires au projet.
- **main**: Contient des scripts auxiliaires ou de configuration.
- **src**: Code source principal du projet.
  - **main.cpp**: Fichier source principal implémentant la logique de contrôle PID et la gestion des servomoteurs.
- **test**: Contient les tests automatisés du projet.
- **.gitignore**: Fichier pour ignorer les fichiers non nécessaires dans git.
- **platformio.ini**: Fichier de configuration pour PlatformIO, spécifie les dépendances et l'environnement de build.

## Configuration Matérielle

- **Arduino Nano**
- **MPU6050 (accéléromètre et gyroscope)**
- **Servomoteurs (x2)**

## Installation

1. Cloner le dépôt Git :
   ```
   git clone [URL_DU_REPO]
   ```
2. Ouvrir le projet avec PlatformIO.
3. Connecter l'Arduino Nano via USB.
4. Compiler et téléverser le programme via PlatformIO.

## Utilisation

Une fois le programme téléversé, le gyropode doit être placé sur une surface plane pour calibrer les capteurs. Après la calibration, inclinez légèrement le gyropode pour commencer à le déplacer. Il ajustera automatiquement sa position pour maintenir l'équilibre.

## Fonctionnalités

- **Contrôle PID**: Ajustement dynamique pour maintenir l'équilibre.
- **Filtrage des données du capteur**: Utilisation de filtres passe-bas pour lisser les lectures des capteurs.
- **Détection et ajustement de la vitesse et de l'angle**: Gestion précise des servomoteurs pour une navigation fluide.

## Dépendances

- Wire (pour la communication I2C)
- Servo (pour le contrôle des servomoteurs)
- I2Cdev et MPU6050 (pour la gestion du capteur)
