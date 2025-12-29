### 🚓 ESP32-S3 Night Patrolling Robot (Firmware)
This repository contains the Arduino firmware for an ESP32-S3 based Night Patrolling Surveillance Robot.
The firmware enables real-time sensing, autonomous navigation, Firebase connectivity, GPS tracking, GSM alerts, and remote control.
## 📌 Firmware Overview
The ESP32-S3 acts as the central controller, integrating multiple sensors and communication modules.
It continuously uploads telemetry to Firebase Realtime Database and responds to commands sent from a web-based command center.
## ⚙️ Key Features
Wi-Fi connectivity with Firebase RTDB
Manual & Autonomous (Auto-Pilot) modes
Smart obstacle avoidance using ultrasonic sensor
Suspicious sound detection (I2S microphone)
Motion / theft detection using MPU6050
GPS-based live location tracking
GSM-based SMS alert system
PWM buzzer alarm system
Servo-based scanning mechanism
Real-time telemetry upload
## 🧠 System Workflow
ESP32 connects to Wi-Fi
Firebase connection is established
Sensors initialize and start monitoring
Telemetry uploads every few seconds
GPS updates live location
Alerts trigger buzzer and SMS
Robot responds to Firebase control commands 
## 🛠️ Required Libraries
Install the following libraries via Arduino Library Manager:
Firebase ESP Client (v4.4.x recommended)
TinyGPSPlus
Adafruit MPU6050
Adafruit Unified Sensor
ESP32Servo
