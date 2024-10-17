// -----
// AcceleratedRotator.ino - Example for the RotaryEncoder library.
// This class is implemented for use with the Arduino environment.
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// 13.11.2019 converted to AcceleratedRotator by Damian Philipp
// 06.02.2021 conditions and settings added for ESP8266
// -----

// This example checks the state of the rotary encoder in the loop() function.
// It then computes an acceleration value and prints the current position when changed.
// There is detailed output given to the Serial.
// You may play around with the constants to fit to your needs.

// Hardware setup:
// Attach a rotary encoder with output pins to
// * 2 and 3 on Arduino UNO.
// * A2 and A3 can be used when directly using the ISR interrupts, see comments below.
// * D5 and D6 on ESP8266 board (e.g. NodeMCU).
// Swap the pins when direction is detected wrong.
// The common contact should be attached to ground.

#include <Arduino.h>
#include "Ticker.h"
#include "RotaryEncoder.h"


#define SW 8
#define DT 9
#define CLK 10

// Setup a RotaryEncoder with 4 steps per latch for the 2 signal input pins:
// RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(DT, CLK, RotaryEncoder::LatchMode::TWO03);
Ticker encoderTicker([]() { encoder.tick(); }, 10, 0, MILLIS);
  

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("AcceleratedRotator example for the RotaryEncoder library.");
  encoderTicker.start();
} // setup()


// Read the current position of the encoder and print out when changed.
void loop() {
  // Update the Ticker in each iteration
  encoderTicker.update();
  
  // Print encoder position every 1000 ms (adjust as needed)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    Serial.print("Position: ");
    Serial.println(encoder.getPosition());
    Serial.print("Velocity: ");
    Serial.println(encoder.getVelocity());
    Serial.print("Acceleration: ");
    Serial.println(encoder.getAcceleration());
    lastPrint = millis();
  }
}
