#include <Wire.h>
#include <SparkFunLSM9DS1_test.h>
#include <ACAN2515.h>
#include "MS5837.h"
#define led 25
#ifndef ARDUINO_ARCH_RP2040
#endif

LSM9DS1 imu;
MS5837 sensor;
const byte PICO_I2C_SDA = 18;
const byte PICO_I2C_SCL = 19;
static const byte MCP2515_SCK  = 2 ; // SCK input of MCP2515 (adapt to your design)
static const byte MCP2515_MOSI = 3 ; // SDI input of MCP2515 (adapt to your design)
static const byte MCP2515_MISO = 4 ; // SDO output of MCP2515 (adapt to your design)
static const byte MCP2515_CS   = 5 ;  // CS input of MCP2515 (adapt to your design)
ACAN2515 can (MCP2515_CS, SPI, 255) ;
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ;
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints 
static unsigned long lastPrint = 0; // Keep track of print time
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO. 

//Function definitions
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);
uint8_t fromFloat(float x);
void setup()
{
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    delay (50) ;
    digitalWrite (led, !digitalRead (led)) ;
  }
  //--- Begin SPI1
  SPI.setSCK (MCP2515_SCK);
  SPI.setTX (MCP2515_MOSI);
  SPI.setRX (MCP2515_MISO);
  SPI.setCS (MCP2515_CS);
  SPI.begin () ;
  Serial.println ("Configure ACAN2515") ;
  Serial.println ("Configure help") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125UL * 1000UL) ; // CAN bit rate 125 kb/s
  // settings.mRequestedMode = ACAN2515Settings::LoopBackMode ; // Select loopback mode
  const uint16_t errorCode = can.begin (settings, NULL) ;
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
  //--- Begin I2C1
  Wire1.setSDA(PICO_I2C_SDA);
  Wire1.setSCL(PICO_I2C_SCL);
  Wire1.begin();
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  if (imu.begin(0x6B, 0x1E, Wire1) == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
  }

}
static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

void loop()
{
  // Update the sensor values whenever new data is available
  can.poll();
  sensor.read();
  float depth = sensor.depth();
  //  Serial.print("Depth: "); // Depth returned in meters (valid for operation in incompressible liquids only. Uses density that is set for fresh or seawater.
  //  Serial.print(sensor.depth());
  //  Serial.println(" m");
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  float gyro_x = imu.calcGyro(imu.gx);
  float gyro_y = imu.calcGyro(imu.gy);
  float gyro_z = imu.calcGyro(imu.gz);
  float accel_x = imu.calcAccel(imu.ax);
  float accel_y = imu.calcAccel(imu.ay);
  float accel_z = imu.calcAccel(imu.az);

  //  if ((lastPrint + PRINT_SPEED) < millis())
  //  {
  //    digitalWrite(led, HIGH);
  //    printGyro();  // Print "G: gx, gy, gz"
  //    printAccel(); // Print "A: ax, ay, az"
  //    printMag();   // Print "M: mx, my, mz"
  //    //     Print the heading and orientation for fun!
  //    //     Call print attitude. The LSM9DS1's mag x and y
  //    //     axes are opposite to the accelerometer, so my, mx are
  //    //     substituted for each other.
  //    printAttitude(imu.ax, imu.ay, imu.az,
  //                  -imu.my, -imu.mx, imu.mz);
  //    Serial.println();
  //    lastPrint = millis(); // Update lastPrint time
  //    digitalWrite(led, LOW);

  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    frame.ext = false ;
    frame.id = 0x00000018 ;
    frame.len = 8 ;
    frame.data [0] = fromFloat(gyro_x);//gx
    frame.data [1] = fromFloat(gyro_y);//gy
    frame.data [2] = fromFloat(gyro_z);//gz
    frame.data [3] = fromFloat(accel_x);//ax
    frame.data [4] = fromFloat(accel_y );//ay
    frame.data [5] = fromFloat(accel_z );//az  
    frame.data [6] = fromFloat(-depth); ;  //depth in metres
    frame.data [7] = 0x00;
    //      for (int i = 0; i < 8; i++)
    //      {
    //        Serial.println(frame.data[i]);
    //      }
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentFrameCount) ;
    } else {
      Serial.println ("Send failure") ;
    }
  }
}
