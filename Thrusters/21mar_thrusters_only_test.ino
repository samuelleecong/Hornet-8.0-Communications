#include <Servo.h>

byte thruster_A_pin = A0;
byte thruster_B_pin = A1;
byte thruster_C_pin = A2;
byte thruster_D_pin = A3;
byte thruster_E_pin = 7;
byte thruster_F_pin = 6;

Servo thruster_A;
Servo thruster_B;
Servo thruster_C;
Servo thruster_D;
Servo thruster_E;
Servo thruster_F;

int soft_kill = 1;
int ks_state;
long pwm[6] = {1550,1550,1550,1550,1550,1550};

void esc_init() {
  Serial.println("INIT");
  thruster_A.writeMicroseconds(1500);
  thruster_B.writeMicroseconds(1500);
  thruster_C.writeMicroseconds(1500);
  thruster_D.writeMicroseconds(1500);
  thruster_E.writeMicroseconds(1500);
  thruster_F.writeMicroseconds(1500);
  delay(3000); // change this for init signal
}


void setup() {
  // put your setup code here, to run once:
  pinMode(9, INPUT);
  pinMode(8, OUTPUT);
  digitalWrite(8, soft_kill);
  thruster_A.attach(thruster_A_pin);
  thruster_B.attach(thruster_B_pin);
  thruster_C.attach(thruster_C_pin);
  thruster_D.attach(thruster_D_pin);
  thruster_E.attach(thruster_E_pin);
  thruster_F.attach(thruster_F_pin);
  while (!digitalRead(9)) {
    delay(1);
  }

  ks_state = 1;
  esc_init();
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  thruster_A.writeMicroseconds(pwm[0]);
  thruster_B.writeMicroseconds(pwm[1]);
  thruster_C.writeMicroseconds(pwm[2]);
  thruster_D.writeMicroseconds(pwm[3]);
  thruster_E.writeMicroseconds(pwm[4]);
  thruster_F.writeMicroseconds(pwm[5]);

  for (int i = 0; i < 6; i++) {
    Serial.print("Thruster ");
    Serial.print(i);
    Serial.print(" PWM value: ");
    Serial.println(pwm[i]);
  }
}
