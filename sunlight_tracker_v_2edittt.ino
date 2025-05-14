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
