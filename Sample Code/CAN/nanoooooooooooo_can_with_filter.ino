//——————————————————————————————————————————————————————————————————————————————
//  ACAN2515 Demo in loopback mode, for the Raspberry Pi Pico
//  Demo sketch that uses SPI1
//——————————————————————————————————————————————————————————————————————————————

#ifndef ARDUINO_ARCH_RP2040
#endif

//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515.h>

//——————————————————————————————————————————————————————————————————————————————
// The Pico has two SPI peripherals, SPI and SPI1. Either (or both) can be used.
// The are no default pin assignments so they must be set explicitly.
// Testing was done with Earle Philhower's arduino-pico core:
// https://github.com/earlephilhower/arduino-pico
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_SCK  = 13 ; // SCK input of MCP2515 (adapt to your design)
static const byte MCP2515_MOSI = 11 ; // SDI input of MCP2515 (adapt to your design)
static const byte MCP2515_MISO = 12 ; // SDO output of MCP2515 (adapt to your design)
static const byte MCP2515_CS   = 10 ;  // CS input of MCP2515 (adapt to your design)
//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, SPI, 255) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 20 MHz

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

static void receive0 (const CANMessage & inMessage) {
  Serial.println ("Receive 0") ;
}

//——————————————————————————————————————————————————————————————————————————————

static void receive1 (const CANMessage & inMessage) {
  Serial.println ("Receive 1") ;
}

//——————————————————————————————————————————————————————————————————————————————

static void receive2 (const CANMessage & inMessage) {
  Serial.println ("Receive 2") ;
}


void setup () {
  //  pinMode(10,OUTPUT);
  //  pinMode(11,OUTPUT);
  //  pinMode(12,OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(3, OUTPUT);
  //--- Switch on builtin led
  //--- Start serial
  Serial.begin (115200) ;
  //--- Wait for serial (blink led at 10 Hz during waiting)
  //  while (!Serial) {
  //    delay (50) ;
  //    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  //  }
  //--- There are no default SPI1 pins so they must be explicitly assigned
  //--- Begin SPI1
  SPI.begin () ;
  //--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  Serial.println ("Configure help") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125UL * 1000UL) ; // CAN bit rate 125 kb/s
  // settings.mRequestedMode = ACAN2515Settings::LoopBackMode ; // Select loopback mode

  //  const ACAN2515Mask rxm0 = extended2515Mask (0x1FFFFFFF) ; // For filter #0 and #1
  //  const ACAN2515Mask rxm1 = standard2515Mask (0x7F0, 0xFF, 0) ; // For filter #2 to #5
  //  const ACAN2515AcceptanceFilter filters [] = {
  //    {extended2515Filter (0x12345678), receive0},
  //    {extended2515Filter (0x18765432), receive1},
  //    {standard2515Filter (0x560, 0x55, 0), receive2}
  //  } ;

  //

  const ACAN2515Mask rxm0 = standard2515Mask (0x7FF, 0, 0) ;
  const ACAN2515AcceptanceFilter filter [] = {
    {standard2515Filter (0x02, 0, 0), receive2},// RXF2
  } ;
  const uint16_t errorCode = can.begin (settings, 
                                        NULL,
                                        rxm0, // Value set to RXM0 register
                                         // Value set to RXM1 register
                                        filter, // The filter array
                                        1) ; // Filter array size

  //  const uint16_t errorCode = can.begin (settings, NULL, rxm0, rxm1, filters, 3) ;
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
    digitalWrite(3, HIGH);
  }
}

//----------------------------------------------------------------------------------------------------------------------

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  //  digitalWrite(10, LOW);
  //  SPI.transfer(0x03);
  //  SPI.transfer(0x28);
  //  Serial.println((int)SPI.transfer(0x2A));
  //  digitalWrite(10, HIGH);
  //  delay(1000);

  can.poll();
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 1500 ;
    frame.ext = false ;
    frame.id = 0x20 ;
    frame.len = 8 ;
    frame.data [0] = 0x01 ;
    frame.data [1] = 0x01 ;
    frame.data [2] = 0x02 ;
    frame.data [3] = 0x02 ;
    frame.data [4] = 0x03 ;
    frame.data [5] = 0x03 ;
    frame.data [6] = 0x02 ;
    frame.data [7] = 0x02 ;
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentFrameCount) ;
      digitalWrite(3, LOW);
    } else {
      Serial.println ("Send failure") ;
      digitalWrite(3, HIGH);
    }
  }
  if (can.receive (frame)) {
    gReceivedFrameCount ++ ;
    Serial.print ("  id: "); Serial.println (frame.id, HEX);
    Serial.print ("  ext: "); Serial.println (frame.ext);
    Serial.print ("  rtr: "); Serial.println (frame.rtr);
    Serial.print ("  len: "); Serial.println (frame.len);
    Serial.print ("  data: ");
    for (int x = 0; x < frame.len; x++) {
      Serial.print (frame.data[x], HEX); Serial.print(":");
    }
    Serial.println ("");
    Serial.print ("Received: ") ;
    Serial.println (gReceivedFrameCount) ;
  }
}


//——————————————————————————————————————————————————————————————————————————————
