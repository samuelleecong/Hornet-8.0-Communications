/* Test code for issue where signals sent in pulses will cause inconsistent directional movement. Original test setup was with 2 buttons for forward and backward movement
When button is tapped (aka ~10ms of signal sent out), the thruster might move backwards when its supposed to go forwards periodically. Ideal delay between pulses is 
~0.5s, backward thrusts doesn't seem to have the same issue.

Possible explaination: inherent behaviour of PMDC motor as it tries to start/stop which may create opposite momentum, backward thrust is also weaker than that of forward
thrust.

*/

#include <Servo.h> 
 
#define button1 11 
#define button2 10 
 
int pwm_init = 1500; 
int pwm2 = 1550; 
int pwm3 = 1450; 
int pwm4 = 1500; 
 
const byte pins[14] = {2, 3, 4, 5, 6, 7, 8, 9, A0, A1, A2, A3, A6, A7}; 
 
int d_pwms[16] = {0, 30, 35, 40, 1560, 1500, 1470, 1460, 1430, 1500}; 
 
Servo thruster_A; 
Servo thruster_B; 
 
void setup() { 
  pinMode(button1, INPUT_PULLUP); 
  pinMode(button2, INPUT_PULLUP); 
  /*for (uint8_t i = 0; i < 14; i++) { 
    thruster[i].attach(pins[i]); 
    } 
 
    for (uint8_t i = 0; i < 14; i++) { 
    thruster[i].writeMicroseconds(pwm_init); 
    }*/ 
  thruster_A.attach(A2); 
  thruster_B.attach(5); 
  thruster_A.writeMicroseconds(A2); 
  thruster_B.writeMicroseconds(5); 
  delay(5000); 
} 
 
void loop() { 
  //  if (!digitalRead(button1)) { 
  //    for (uint8_t i = 0; i < 14; i++){ 
  //      thruster[i].writeMicroseconds(pwm2); 
  //    } 
  //  } else if (!digitalRead(button2)) { 
  //    for (uint8_t i = 0; i < 14; i++){ 
  //      thruster[i].writeMicroseconds(pwm3); 
  //    } 
  //  } else { 
  //    for (uint8_t i = 0; i < 14; i++){ 
  //      thruster[i].writeMicroseconds(pwm4); 
  //    } 
  //  } 
  //  delay(1); 
  //  for (uint8_t i = 0; i < 14; i++){ 
  //    thruster[i].writeMicroseconds(pwm2); 
  //  } 
  //  delay(500); 
  //  for (uint8_t i = 0; i < 14; i++){ 
  //    thruster[i].writeMicroseconds(pwm4); 
  //  } 
  //  delay(500); 
 /* Increasing speed in forward direction */
  for (int j = 0; j <= 30; j = j + 2) { 
    thruster_A.writeMicroseconds(1525 + j); 
    thruster_B.writeMicroseconds(1525 + j); 
    delay(100); 
  } 
 /* Stop for .5sec */
  thruster_A.writeMicroseconds(1500); 
  thruster_B.writeMicroseconds(1500); 
  delay(500); 
 /* Increasing speed in reverse direction */
  for (int i = 0; i <= 30; i += 2)  { 
    thruster_A.writeMicroseconds(1475 - i); 
    thruster_B.writeMicroseconds(1475 - i); 
    delay(100); 
  } 
}
