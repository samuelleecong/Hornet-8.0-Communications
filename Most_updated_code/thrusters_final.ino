//#ifndef ARDUINO_ARCH_RP2040
//#endif

#include <ACAN2515.h>

#include <Servo.h>

Servo thruster_A;
Servo thruster_B;
Servo thruster_C;
Servo thruster_D;
Servo thruster_E;
Servo thruster_F;

byte thruster_pins[6] = {A0, A1, A2, A3, 7, 6};
Servo thrusters[6] = {thruster_A, thruster_B, thruster_C, thruster_D, thruster_E, thruster_F};

int soft_kill;
int ks_state;
int pwm[6];

static const byte MCP2515_SCK  = 13 ; // SCK input of MCP2515 (adapt to your design)
static const byte MCP2515_MOSI = 11 ; // SDI input of MCP2515 (adapt to your design)
static const byte MCP2515_MISO = 12 ; // SDO output of MCP2515 (adapt to your design)
static const byte MCP2515_CS   = 10 ;  // CS input of MCP2515 (adapt to your design)

ACAN2515 can(MCP2515_CS, SPI, 255);


static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 20 MHz
static uint32_t gBlinkLedDate = 0;
static uint32_t gReceivedFrameCount = 0;
static uint32_t gSentFrameCount = 0;

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
  Serial.begin(115200);
  
  Serial.println("INIT Start");
  soft_kill = 1;
  pinMode(9, INPUT);
  pinMode(8, OUTPUT);
  digitalWrite(8, soft_kill);

  while (!digitalRead(9));

  for (int i = 0; i < 6; i++)
    thrusters[i].attach(thruster_pins[i]);

  ks_state = 1;
  esc_init();


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
  Serial.println("INIT DONE");
}

void loop() {
  // can.poll();
  can.isr_core();

  // KS CHECK
  if (!digitalRead(9)) {
    ks_state = 0;
    Serial.println("KILLSWITCH PULLED");
  }

  if (!ks_state && digitalRead(9)) {
    ks_state = 1;
    esc_init();
  }

  // CAN SEND
  CANMessage frame ;

  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 1000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    frame.ext = false ;
    frame.id = 0x20 ;
    frame.len = 8 ;
    // CAN FRAME {KS STATE, RAINDROP STATE, MSG_COUNT,0,0,0,0,0}
    frame.data [0] = ks_state;
    frame.data [1] = 0x00 ;
    frame.data [2] = gSentFrameCount;
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
    for (int x = 0; x < frame.len - 1; x++) {
      Serial.print (frame.data[x], HEX); Serial.print(":");
    }
    Serial.println (frame.data[frame.len - 1], HEX);
    Serial.print ("Received: ") ;
    Serial.println (gReceivedFrameCount) ;

    if (frame.id == 2) {
      // CHECK SOFT KILL
      if (frame.data[6])
        soft_kill = 0;
      else
        soft_kill = 1;
      digitalWrite(8, soft_kill);

      // MAP PWM
      for (int i = 0; i < 6; i++) {
        pwm[i] = map((int) frame.data[i], 0, 255, 1100, 1900);
        thrusters[i].writeMicroseconds(pwm[i]);
      }
    }
  }

  for (int i = 0; i < 6; i++) {
    Serial.print("Thruster ");
    Serial.print(i);
    Serial.print(" PWM value: ");
    Serial.print(pwm[i]);
    Serial.print(" ");
    Serial.println(thrusters[i].readMicroseconds());
  }

}
