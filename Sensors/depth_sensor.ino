/* Blue Robotics MS5837 Library Example
-----------------------------------------------------
 
Title: Blue Robotics MS5837 Library Example

Description: This example demonstrates the MS5837 Library with a connected
sensor. The example reads the sensor and prints the resulting values
to the serial terminal.

The code is designed for the Arduino Uno board and can be compiled and 
uploaded via the Arduino 1.0+ software.

-------------------------------
The MIT License (MIT)

Copyright (c) 2015 Blue Robotics Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/

#include <Wire.h>
#include "MS5837.h"

/* Define the set of PICO's I2C pins, GPIO pins 16 and 17. */
const byte sda = 16; 
const byte scl = 17;

MS5837 sensor;

void setup() {
  
  Serial.begin(9600);
  Serial.println("Starting");
  pinMode(25, OUTPUT); // built-in LED at pin 25, light up to indicate program is running
  digitalWrite(25, HIGH);
  Wire.setSDA(sda);
  Wire.setSCL(scl);
  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (997 for freshwater, 1029 for seawater)
}

void loop() {
  // Update pressure and temperature readings
  sensor.read(); // The read from I2C takes up for 40 ms, so use sparingly if possible

  Serial.print("Pressure: "); // Pressure returned in mbar or mbar*conversion rate.
  Serial.print(sensor.pressure()); 
  Serial.println(" mbar");
  
  Serial.print("Temperature: "); // Temperature returned in deg C.
  Serial.print(sensor.temperature()); 
  Serial.println(" deg C");
  
  Serial.print("Depth: "); // Depth returned in meters (valid for operation in incompressible liquids only. Uses density that is set for fresh or seawater.
  Serial.print(sensor.depth()); 
  Serial.println(" m");
  
  Serial.print("Altitude: "); // Altitude returned in meters (valid for operation in air only).
  Serial.print(sensor.altitude()); 
  Serial.println(" m above mean sea level");

  delay(1000);
}
