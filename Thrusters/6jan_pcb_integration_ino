#ifndef ARDUINO_ARCH_RP2040
#endif

#include <ACAN2515.h>

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

int ks_state;
long pwm[6];

static const byte MCP2515_SCK  = 13 ; // SCK input of MCP2515 (adapt to your design)
static const byte MCP2515_MOSI = 11 ; // SDI input of MCP2515 (adapt to your design)
static const byte MCP2515_MISO = 12 ; // SDO output of MCP2515 (adapt to your design)
static const byte MCP2515_CS   = 10 ;  // CS input of MCP2515 (adapt to your design)

ACAN2515 can(MCP2515_CS, SPI, 255);


static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 20 MHz
static uint32_t gBlinkLedDate = 0;
static uint32_t gReceivedFrameCount = 0;
static uint32_t gSentFrameCount = 0;

void esc_innit() {
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
  digitalWrite(8, HIGH);
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
  esc_innit();
  Serial.begin(115200);
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  //--- There are no default SPI1 pins so they must be explicitly assigned
  //--- Begin SPI1
  SPI.begin () ;
  //--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125UL * 1000UL) ; // CAN bit rate 125 kb/s
  // settings.mRequestedMode = ACAN2515Settings::LoopBackMode ; // Select loopback mode
  const uint16_t errorCode = can.begin(settings, NULL);
  //const uint16_t errorCode = can.begin (settings, [] {}) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Propagation Segment: ") ;
    Serial.println (settings.mPropagationSegment) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW: ") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling: ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  } else {
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

void loop() {
  // KS CHECK
  if (!digitalRead(9)) {
    ks_state = 0;
    Serial.println("KILLSWITCH PULLED");
  }
  if (!ks_state && digitalRead(9)) {
    ks_state = 1;
    esc_innit();
  }

  // CAN SEND
  can.poll();
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    frame.ext = true ;
    frame.id = 0x1FFFFFAD ;
    frame.len = 8 ;
    // CAN FRAME {KS STATE, RAINDROP STATE, 0,0,0,0,0,0}
    if (ks_state)
      frame.data [0] = 0x01 ;
    else
      frame.data[0] = 0x00;
    frame.data [1] = 0x00 ;
    frame.data [2] = 0x00 ;
    frame.data [3] = 0x00 ;
    frame.data [4] = 0x00 ;
    frame.data [5] = 0x00 ;
    frame.data [6] = 0x00 ;
    frame.data [7] = 0x00 ;
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentFrameCount) ;
    } else {
      Serial.println ("Send failure") ;
    }
  }

  // CAN RECEIVE
  if (can.receive (frame)) {
    gReceivedFrameCount ++ ;
    Serial.print ("  id: "); Serial.println (frame.id, HEX);
    Serial.print ("  ext: "); Serial.println (frame.ext);
    Serial.print ("  rtr: "); Serial.println (frame.rtr);
    Serial.print ("  len: "); Serial.println (frame.len);
    Serial.print ("  data: ");
    /*for(int x = 0; x < frame.len; x++) {
      Serial.print (frame.data[x],HEX); Serial.print(":");
      }
      Serial.println ("");*/
    Serial.print ("Received: ") ;
    Serial.println (gReceivedFrameCount) ;
  }
  // MAP PWM
  for (int i = 0; i < 6; i++) {
    if (pwm[i] != 0) {
      int val = int(frame.data[i]);
      pwm[i] = map(val, 0, 255, 1000, 2000);
      Serial.print("Thruster ");
      Serial.print(i);
      Serial.print(" PWM value: ");
      Serial.println(pwm[i]);
    }
  }
  thruster_A.writeMicroseconds(pwm[0]);
  thruster_B.writeMicroseconds(pwm[1]);
  thruster_C.writeMicroseconds(pwm[2]);
  thruster_D.writeMicroseconds(pwm[3]);
  thruster_E.writeMicroseconds(pwm[4]);
  thruster_F.writeMicroseconds(pwm[5]);
}
