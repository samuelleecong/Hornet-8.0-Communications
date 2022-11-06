#include <Servo.h>
byte thruster_A_pin = 9;
Servo thruster_A;

void setup() {
  // put your setup code here, to run once:
  // pinMode(9,OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  thruster_A.attach(thruster_A_pin);
  thruster_A.writeMicroseconds(1500);
  delay(5000);

}

void loop() {
  // put your main code here, to run repeatedly:
  thruster_A.writeMicroseconds(1550);
}
