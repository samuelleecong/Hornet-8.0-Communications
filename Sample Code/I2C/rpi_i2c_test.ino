#include <Wire.h>
const int sda = 16;
const int scl = 17;
void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);
  //open serial monitor
  while(!Serial);
  Wire.setSDA(sda);
  Wire.setSCL(scl);
  Wire.begin();
  pinMode(25,OUTPUT);
  pinMode(20,OUTPUT);
  pinMode(19,OUTPUT);
 
}

void loop() {
   digitalWrite(25,HIGH);
   digitalWrite(20,HIGH);
   analogWrite(19,200);
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(8); // transmit to device #4
  Wire.write('a');     // sends five bytes              // sends one byte  
  Serial.println("sent");
  Wire.endTransmission();    // stop transmitting
  delay(500);
}
