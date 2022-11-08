#ifndef ARDUINO_ARCH_RP2040
#endif

#include <ACAN2515.h>
#include <Servo.h>

byte thruster_A_pin = 9;
Servo thruster_A;

static const byte MCP2515_SCK = 13;   // SCK input of MCP2515 (adapt to your design)
static const byte MCP2515_MOSI = 11;  // SDI input of MCP2515 (adapt to your design)
static const byte MCP2515_MISO = 12;  // SDO output of MCP2515 (adapt to your design)

static const byte MCP2515_CS = 10;  // CS input of MCP2515 (adapt to your design)
static const byte MCP2515_INT = 2;  // INT output of MCP2515 (adapt to your design)

ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;  // 20 MHz
static uint32_t gBlinkLedDate = 0;
static uint32_t gReceivedFrameCount = 0;
static uint32_t gSentFrameCount = 0;

void setup() {
  // put your setup code here, to run once:
  // pinMode(9, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  thruster_A.attach(thruster_A_pin);
  thruster_A.writeMicroseconds(1500);
  delay(5000);
  // Start CAN
  //--- Start serial
  Serial.begin(115200);
  //--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay(50);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  //--- There are no default SPI1 pins so they must be explicitly assigned
  //--- Begin SPI1
  SPI.begin();
  //--- Configure ACAN2515
  Serial.println("Configure ACAN2515");
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 125UL * 1000UL);  // CAN bit rate 125 kb/s
  // settings.mRequestedMode = ACAN2515Settings::LoopBackMode ; // Select loopback mode
  const uint16_t errorCode = can.begin(settings, [] {
    can.isr();
  });
  //const uint16_t errorCode = can.begin (settings, [] {}) ;
  if (errorCode == 0) {
    Serial.print("Bit Rate prescaler: ");
    Serial.println(settings.mBitRatePrescaler);
    Serial.print("Propagation Segment: ");
    Serial.println(settings.mPropagationSegment);
    Serial.print("Phase segment 1: ");
    Serial.println(settings.mPhaseSegment1);
    Serial.print("Phase segment 2: ");
    Serial.println(settings.mPhaseSegment2);
    Serial.print("SJW: ");
    Serial.println(settings.mSJW);
    Serial.print("Triple Sampling: ");
    Serial.println(settings.mTripleSampling ? "yes" : "no");
    Serial.print("Actual bit rate: ");
    Serial.print(settings.actualBitRate());
    Serial.println(" bit/s");
    Serial.print("Exact bit rate ? ");
    Serial.println(settings.exactBitRate() ? "yes" : "no");
    Serial.print("Sample point: ");
    Serial.print(settings.samplePointFromBitStart());
    Serial.println("%");
  } else {
    Serial.print("Configuration error 0x");
    Serial.println(errorCode, HEX);
  }
}

void loop() {
  if (digitalRead(7) == LOW) {
    Serial.println("KILLSWITCH PULLED");
  }
  // put your main code here, to run repeatedly
  CANMessage frame;
  if (can.receive(frame)) {
    gReceivedFrameCount++;
    Serial.print("  id: ");
    Serial.println(frame.id, HEX);
    Serial.print("  ext: ");
    Serial.println(frame.ext);
    Serial.print("  rtr: ");
    Serial.println(frame.rtr);
    Serial.print("  len: ");
    Serial.println(frame.len);
    Serial.print("  data: ");
    for (int x = 0; x < frame.len; x++) {
      Serial.print(frame.data[x], HEX);
      Serial.print(":");
    }
    int pwm = int(frame.data[0]);
    //int pwm = int(0xAF);
    int val = map(pwm, 0, 255, 0, 2000);
    thruster_A.writeMicroseconds(val);
    Serial.println("");
    Serial.print("Received: ");
    Serial.println(gReceivedFrameCount);
    Serial.println(pwm);
    Serial.println(val);
  }
}