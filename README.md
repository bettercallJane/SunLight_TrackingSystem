# ☀️SunLight_TrackingSystem
A guide to establishing a dual-axis smart system for tracking sunlight


A comprehensive guide to building a **dual-axis smart tracking system** designed to follow the sun’s position in real-time, maximizing solar panel efficiency or supporting solar-based experiments.

---

## 📖 Overview

This project outlines the design, components, and programming needed to create a **dual-axis solar tracker** that intelligently follows the sun throughout the day using sensors, actuators, and a microcontroller. Unlike single-axis trackers, this system adjusts **both azimuth and elevation**, improving sunlight exposure significantly.

---

## 🔧 Features

- ✅ Dual-axis motion (horizontal + vertical tracking)
- ✅ Real-time light intensity sensing
- ✅ Automated adjustment using metal servo motors
- ✅ Energy-efficient design using low-power components
- ✅ Optional IoT integration for data logging or remote control

---

## 🧰 Components Required

| Component                | Description                                                                  |
|--------------------------|------------------------------------------------------------------------------|
| Microcontroller          | Arduino Uno / ESP32 / Raspberry Pi                                           |
| Metal Servo Motors (x2)  | For azimuth and elevation control                                            |
| Light Sensors (x4)       | LDRs or photodiodes for sun detection                                        |
| Solar Panel              | For real energy generation                                                   |
| Breadboard & Wires       | Circuit prototyping                                                          |
| Power Supply             | 5V or USB + External Supply for Metal Servo, recharged from Stirling Engine  |


---

## 📐 System Design

The tracker uses **four LDRs** arranged in a cross pattern to detect sunlight intensity differences. Based on this data, the microcontroller calculates adjustments and moves the platform via two servo motors.

---

 ## How It Works

1. Sensors detect sunlight from different directions.
2. The controller compares sensor readings.
3. Motors adjust the panel's angle until sensors are balanced (i.e., facing the sun).
4. System repeats periodically.

---

##💻 Code 

```cpp
#include <Servo.h>

// Initializing the pins
Servo horizontal; // Pin 2
Servo vertical;   // Pin 13


const int servohoriLimitHigh = 105;
const int servohoriLimitLow = 75;

const int servovertLimitHigh = 50;
const int servovertLimitLow = 20;

// LDR Pins
const int ldrlt = A0; // Top Left
const int ldrrt = A3; // Top Right
const int ldrld = A1; // Bottom Left
const int ldrrd = A2; // Bottom Right

// LDR Averaging
const int numReadings = 5;
int readings[4][numReadings] = {0};
int readIndex = 0;
int total[4] = {0};
int average[4] = {0};

// Initializing Servo Positions 
int servohori = 90;
int servovert = 50;

void setup() {
  Serial.begin(9600);
  horizontal.attach(2);
  vertical.attach(13);
  horizontal.write(servohori);
  vertical.write(servovert);
  delay(2500); // Initial settling time
}

void loop() {
  readLDR();

  int lt = average[0]; // Top left
  int rt = average[1]; // Top right
  int ld = average[2]; // Bottom left
  int rd = average[3]; // Bottom right

  // Calculate light averages
  int avt = (lt + rt) / 2; // Top avg
  int avd = (ld + rd) / 2; // Bottom avg
  int avl = (lt + ld) / 2; // Left avg
  int avr = (rt + rd) / 2; // Right avg

  // Differences (error signals)
  int dvert = avt - avd;
  int dhoriz = avl - avr;

  // Tolerance threshold
  const int tol = 90;

  // Vertical Movement
  if (abs(dvert) > tol) {
    int stepSize = map(abs(dvert), tol, 1023, 1, 10); // Dynamic step size
    if (avt > avd && servovert < servovertLimitHigh) {
      servovert += stepSize;
    } 
    else if (avd > avt && servovert > servovertLimitLow) {
      servovert -= stepSize;
    }
    
    servovert = constrain(servovert, servovertLimitLow, servovertLimitHigh);
    vertical.write(servovert);
    delay(20); 
  }

  // Horizontal Movement
  if (abs(dhoriz) > tol) {
    int stepSize = map(abs(dhoriz), tol, 1023, 1, 10);
    if (avl > avr && servohori > servohoriLimitLow) {
      servohori -= stepSize;
    } 
    else if (avr > avl && servohori < servohoriLimitHigh) {
      servohori += stepSize;
    }
    
    servohori = constrain(servohori, servohoriLimitLow, servohoriLimitHigh);
    horizontal.write(servohori);
    delay(20);
  }

  // Debugging (Optional)
  Serial.print("H: "); Serial.print(servohori);
  Serial.print(" V: "); Serial.print(servovert);
  Serial.print(" LDRs: ");
  Serial.print(lt); Serial.print(","); Serial.print(rt);
  Serial.print(","); Serial.print(ld); Serial.print(","); Serial.println(rd);

  delay(10); // Main loop delay
}

void readLDR() {
  for (int i = 0; i < 4; i++) {
    total[i] -= readings[i][readIndex];
  }

  readings[0][readIndex] = analogRead(ldrlt);
  readings[1][readIndex] = analogRead(ldrrt);
  readings[2][readIndex] = analogRead(ldrld);
  readings[3][readIndex] = analogRead(ldrrd);

  for (int i = 0; i < 4; i++) {
    total[i] += readings[i][readIndex];
    average[i] = total[i] / numReadings;
  }

  readIndex = (readIndex + 1) % numReadings;
}
