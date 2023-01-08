#include <Servo.h>
Servo act;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  act.attach(20);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i =0; i< 180; i+=5){
    act.write(i);
    delay(200);
  }
  delay(1000);

}
