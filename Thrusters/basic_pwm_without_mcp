#include <Servo.h>

byte thruster_A_pin = 3;
byte thruster_B_pin = 5;
byte thruster_C_pin = 6;
byte thruster_D_pin = 9;

Servo thruster_A;
Servo thruster_B;
Servo thruster_C;
Servo thruster_D;

int ks_state;
long pwm = 1565;

void esc_innit() {
  thruster_A.writeMicroseconds(1500);
  thruster_B.writeMicroseconds(1500);
  thruster_C.writeMicroseconds(1500);
  thruster_D.writeMicroseconds(1500);
  delay(3000); // change this for init signal
}


void setup() {
  // put your setup code here, to run once:
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  thruster_A.attach(thruster_A_pin);
  thruster_B.attach(thruster_B_pin);
  thruster_C.attach(thruster_C_pin);
  thruster_D.attach(thruster_D_pin);
  while (!digitalRead(7)) {
    delay(10);
  }
  
  ks_state = 1;
  esc_innit();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(7) == LOW) {
    ks_state = 0;
    Serial.println("KILLSWITCH PULLED");
  }
  if (ks_state == 0 && digitalRead(7)) {
    ks_state = 1;
    esc_innit();
  }
  thruster_A.writeMicroseconds(pwm);
  thruster_B.writeMicroseconds(pwm);
  thruster_C.writeMicroseconds(pwm);
  thruster_D.writeMicroseconds(pwm);
}
