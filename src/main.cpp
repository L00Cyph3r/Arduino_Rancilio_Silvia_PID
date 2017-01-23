/**
 *
 *
 */
#include "Arduino.h"
#include "PID_v1.h"

#define pinTempBoiler     A0
#define pinTempGroupHead  A1
#define pinRelayHeater    LED_BUILTIN
#define pinMaster         12


int boilerPower = 1000; // Watts
float boilerVolume = 300; // Grams

unsigned int windowSize = 1000 / 1;
unsigned long windowStartTime;
double acceleration = 1;
double setPoint, Input, Output;

double aggKp=50 / acceleration;
// double aggKi=5 * (windowSize / 1000);


double aggKi=0.1 / acceleration;
double aggKd=25 / acceleration;

PID bPID(&Input, &Output, &setPoint, aggKp,aggKi,aggKd, DIRECT);

void setup() {
        pinMode(pinRelayHeater, OUTPUT);
        Input = 20.0;
        Serial.begin(9600);
        windowStartTime = millis();
        setPoint = 93.0;
        bPID.SetSampleTime(windowSize);
        bPID.SetOutputLimits(0, windowSize);
        bPID.SetMode(AUTOMATIC);
        Serial.println();
        Serial.println("Starting!");
}

void loop() {
        // Input = (analogRead(pinTempBoiler)/1024.0)*5.0 * 100;
        bPID.Compute();
        if (millis() - windowStartTime > windowSize) {
                // Serial.print("T:\t");     Serial.print(millis());
                // Serial.print("\tInput:\t");     Serial.print(Input);
                // Serial.print("\tOutput:\t");  Serial.print(Output,0);
                // Serial.print("\tP:\t");       Serial.print(bPID.GetKp(), 3);
                // Serial.print("\tI:\t");       Serial.print(bPID.GetKi(), 3);
                // Serial.print("\tD:\t");       Serial.println(bPID.GetKd(), 3);
                Serial.print(millis());
                Serial.print(";"); Serial.print(Input);
                Serial.print(";"); Serial.print(Output,0);
                Serial.print(";"); Serial.print(bPID.GetKp(), 3);
                Serial.print(";"); Serial.print(bPID.GetKi(), 3);
                Serial.print(";"); Serial.println(bPID.GetKd(), 3);
                // Serial.println(digitalRead(pinRelayHeater));
                windowStartTime += windowSize;
                Input = Input + ((boilerPower / (4.184 * boilerVolume) * (Output / windowSize)));
                Input = Input + ((0.0004375 * (20 - Input)));

                if (millis() > 120000 && millis() < 180000) {
                        Input = Input - ((boilerPower / (4.184 * boilerVolume)));
                }

                if (millis() > 300000) {
                  Serial.println("quitting!");
                  delay(1000);
                  exit(0);
                }
        }
        if (Output < millis() - windowStartTime) {
                digitalWrite(pinRelayHeater, LOW);
                //Serial.println("Power off!");
        } else {
                digitalWrite(pinRelayHeater, HIGH);
                //Serial.println("Power on!");
        }
        // delay(20);
}
