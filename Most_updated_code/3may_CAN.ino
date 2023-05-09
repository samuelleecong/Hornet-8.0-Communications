#include <Wire.h>
#include <SparkFunLSM9DS1_test.h>
#include <ACAN2515.h>
#include "MS5837.h"
#define led 25
#ifndef ARDUINO_ARCH_RP2040
#endif

LSM9DS1 imu;
MS5837 sensor;
/** USING WIRE0 PINS, MAKE SURE TO DECLARE ALL PINS IN LIB AS WIRE INSTEAD OF WIRE1 IN THE LIBRARY**/
const byte PICO_I2C_SDA = 16;
const byte PICO_I2C_SCL = 17;
static const byte MCP2515_SCK  = 2 ; // SCK input of MCP2515 (adapt to your design)
static const byte MCP2515_MOSI = 3 ; // SDI input of MCP2515 (adapt to your design)
static const byte MCP2515_MISO = 4 ; // SDO output of MCP2515 (adapt to your design)
static const byte MCP2515_CS   = 5 ;  // CS input of MCP2515 (adapt to your design)
ACAN2515 can (MCP2515_CS, SPI, 255) ;
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ;
// VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto
//The filter will produce meaningless results if these data are not correct

//Gyro scale 245 dps convert to radians/sec and offsets
float Gscale = (M_PI / 180.0) * 0.00875; //245 dps scale sensitivity = 8.75 mdps/LSB
int G_offset[3] = {63, 4, 24};

//Accel scale 16457.0 to normalize
float A_B[3]
{  -398.58,  114.45,  144.64};

float A_Ainv[3][3]
{ {  1.05694,  0.02078,  0.01985},
  {   0.02078,  1.05169, -0.02241},
  {  0.01985,  -0.02241,  0.95066}
};

//Mag scale 3746.0 to normalize
float M_B[3]
{ 410.29,  718.04, 599.47};

float M_Ainv[3][3]
{ {  1.00976,  0.01770, 0.03166},
  {  0.01770,  1.05417,  -0.00515},
  { -0.03166,  -0.00515,  1.06969}
};

// local magnetic declination in degrees
float declination = 0.07;

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized. Ki is not used.
#define Kp 10.0
#define Ki 0.0

unsigned long now = 0, last = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds

#define PRINT_SPEED 300 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output

//Function definitions
//void printGyro();
//void printAccel();
//void printMag();
//void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

void float_to_bytes(float val, byte *ptr) {
  union {
    float val;
    byte arr[4];
  } u;
  u.val = val;
  memcpy(ptr, u.arr, 4);
}



void setup()
{
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  //  while (!Serial) {
  //    delay (50) ;
  //    digitalWrite (led, !digitalRead (led)) ;
  //  }
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
  Wire.setSDA(PICO_I2C_SDA);
  Wire.setSCL(PICO_I2C_SCL);
  Wire.begin();
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  if (imu.begin(0x6B, 0x1E, Wire) == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
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
  static char updated = 0; //flags for sensor updates
  static int loop_counter = 0; //sample & update loop counter
  static float Gxyz[3], Axyz[3], Mxyz[3]; //centered and scaled gyro/accel/mag data

  // Update the sensor values whenever new data is available
  if ( imu.accelAvailable() ) {
    updated |= 1;  //acc updated
    imu.readAccel();
  }
  if ( imu.magAvailable() ) {
    updated |= 2; //mag updated
    imu.readMag();
  }
  if ( imu.gyroAvailable() ) {
    updated |= 4; //gyro updated
    imu.readGyro();
  }
  if (updated == 7) //all sensors updated?
  {
    updated = 0; //reset update flags
    loop_counter++;
    get_scaled_IMU(Gxyz, Axyz, Mxyz);

    // correct accel/gyro handedness
    // Note: the illustration in the LSM9DS1 data sheet implies that the magnetometer
    // X and Y axes are rotated with respect to the accel/gyro X and Y, but this is not case.

    Axyz[0] = -Axyz[0]; //fix accel/gyro handedness
    Gxyz[0] = -Gxyz[0]; //must be done after offsets & scales applied to raw data

    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                           Mxyz[0], Mxyz[1], Mxyz[2], deltat);


    // Define Tait-Bryan angles, strictly valid only for approximately level movement
    // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
    // this code corrects for magnetic declination.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order.
    //
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.

    // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
    // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
    roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw   *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    // http://www.ngdc.noaa.gov/geomag-web/#declination
    //conventional nav, yaw increases CW from North, corrected for local magnetic declination

    yaw = -(yaw + declination);
    if (yaw < 0) yaw += 360.0;
    if (yaw >= 360.0) yaw -= 360.0;

    Serial.print("ypr ");
    Serial.print(yaw);
    Serial.print(", ");
    Serial.print(pitch);
    Serial.print(", ");
    Serial.print(roll);
    //      Serial.print(", ");  //prints 24 in 300 ms (80 Hz) with 16 MHz ATmega328
    //      Serial.print(loop_counter);  //sample & update loops per print interval
    loop_counter = 0;
    Serial.println();
    //lastPrint = millis(); // Update lastPrint time
  }
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 100 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    frame.ext = false ;
    frame.id = 0x00000018 ;
    frame.len = 8 ;
    float_to_bytes(roll, frame.data);
    float_to_bytes(pitch, frame.data + 4);
    //    frame.data [0] =
    //    frame.data [1] =
    //    frame.data [2] =
    //    frame.data [3] =
    //    frame.data [4] =
    //    frame.data [5] =
    //    frame.data [6] =
    //    frame.data [7] =
    //          for (int i = 0; i < 8; i++)
    //          {
    //            Serial.println(frame.data[i]);
    //          }
    bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentFrameCount) ;
    } else {
      Serial.println ("Send failure") ;
    }
    frame.ext = false ;
    frame.id = 0x00000019 ;
    frame.len = 8 ;
    float_to_bytes(yaw, frame.data);
    frame.data[4] = byte(max(0, depth * 51));
    frame.data[5] = byte(depth * 51);
    //    frame.data[4] = byte((accel_x + 2) * 64);
    //    frame.data[5] = byte((accel_y + 2) * 64);
    //    frame.data[6] = byte((accel_z + 2) * 64);
    //    frame.data [0] = fromFloat(gyro_x);//gx
    //    frame.data [1] = fromFloat(gyro_y);//gy
    //    frame.data [2] = fromFloat(gyro_z);//gz
    //    frame.data [3] = fromFloat(accel_x);//ax
    //    frame.data [4] = fromFloat(accel_y );//ay
    //    frame.data [5] = fromFloat(accel_z );//az
    //    frame.data [6] = fromFloat(-depth); ;  //depth in metres
    //    frame.data [7] = 0x00;
    //          for (int i = 0; i < 8; i++)
    //          {
    //            Serial.println(frame.data[i]);
    //          }
    ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentFrameCount) ;
    } else {
      Serial.println ("Send failure") ;
    }
  }
}

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];
  Gxyz[0] = Gscale * (imu.gx - G_offset[0]);
  Gxyz[1] = Gscale * (imu.gy - G_offset[1]);
  Gxyz[2] = Gscale * (imu.gz - G_offset[2]);

  Axyz[0] = imu.ax;
  Axyz[1] = imu.ay;
  Axyz[2] = imu.az;
  Mxyz[0] = imu.mx;
  Mxyz[1] = imu.my;
  Mxyz[2] = imu.mz;

  //apply accel offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  //observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;

  //update quaternion with integrated contribution
  // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
  gx = gx * (0.5 * deltat); // pre-multiply common factors
  gy = gy * (0.5 * deltat);
  gz = gz * (0.5 * deltat);
  float qa = q1;
  float qb = q2;
  float qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
