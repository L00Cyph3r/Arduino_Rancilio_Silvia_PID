/**
 *
 *
 */
#include "Arduino.h"
#include "PID_v1.h"

#define pinTempBoiler     A0
#define pinTempGroupHead  A1
#define pinRelayHeater    13
#define pinMaster         12

double setPoint, Input, Output;

double aggKp=1;
double aggKi=1;
double aggKd=1;

PID bPID(&Input, &Output, &setPoint, aggKp,aggKi,aggKd, DIRECT);

unsigned int windowSize = 5000;
unsigned long windowStartTime;

void setup() {
  pinMode(pinRelayHeater, OUTPUT);

  Serial.begin(9600);
  windowStartTime = millis();
  setPoint = 93.0;
  bPID.SetOutputLimits(0, windowSize);
  bPID.SetMode(AUTOMATIC);
}

void loop() {
  Input = (analogRead(pinTempBoiler)/1024.0)*5.0 * 100;
  bPID.Compute();
  if (millis() - windowStartTime > windowSize) {
  Serial.print("Input:  "); Serial.println(Input);
  Serial.print("Output: "); Serial.println(Output);
  Serial.println(digitalRead(pinRelayHeater));
    windowStartTime += windowSize;
  }
  if (Output < millis() - windowStartTime) {
    digitalWrite(pinRelayHeater, HIGH);
    //Serial.println("Power on!");
  } else {
    digitalWrite(pinRelayHeater, LOW);
    //Serial.println("Power off!");
  }
  // delay(20);
}
