
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
// calibrated acc & magnetometer
#include <Wire.h>
#include <SparkFunLSM9DS1.h> //change to the new lib name

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
// #define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
// #define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////

#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION +0.08 // Declination (degrees) in Singapore

//Function definitions
void printGyro();
void printAccel();
void printMag();
void setup()
{
  Serial.begin(115200);

  Wire.begin();

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }

}

void loop()
{
  float acc_measured[3]={{0},{0},{0}};
  float acc_bias[3]={{-0.030930},{0.007120},{0.013532}};
  float acc_A_inv[3][3]={{1.000961,0.058052,0.000985},{0.058052,1.000655,0.002663},{0.000985,0.002663,0.992190}};
  float acc_calibrated[3]={{0},{0},{0}};
  
  float mag_measured[3]={{0},{0},{0}};
  float mag_bias[3]={{0.098466},{0.043553},{0.077805}};
  float mag_A_inv[3][3]={{1.117290,0.046215,0.011349},{0.046215,1.103788,0.056327},{0.011349,0.056327,1.112783}};
  float mag_calibrated[3]={{0},{0},{0}};
  int i, j;
  // Update the sensor values whenever new data is available
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
    acc_measured={{(imu.calcAccel(imu.ax), 2)},{(imu.calcAccel(imu.ay), 2)},{(imu.calcAccel(imu.az), 2)}};
    for (i=0;i<3;i++) {
      for(i=0;j<3;j++)
      {
        acc_calibrated[i]+=acc_A_inv[i][j]*(acc_measured[j]-acc_bias[j]);
      }
    }
 
  }
  
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
    mag_measured={{(imu.calcMag(imu.ax), 2)},{(imu.calcMag(imu.ay), 2)},{(imu.calcMag(imu.az), 2)}};
    for (i=0;i<3;i++) {
      for(i=0;j<3;j++)
      {
        mag_calibrated[i]+=mag_A_inv[i][j]*(mag_measured[j]-mag_bias[j]);
      }
    }
  }

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    Serial.println();

    lastPrint = millis(); // Update lastPrint time
  }
}

void printGyro() //still uncalibrated at this pt
{
 // Now we can use the gx, gy, and gz variables as we please.
 // Either print them as raw ADC values, or calculated in DPS.
 Serial.print(imu.calcGyro(imu.gx), 2);
 Serial.print(", ");
 Serial.print(imu.calcGyro(imu.gy), 2);
 Serial.print(", ");
 Serial.print(imu.calcGyro(imu.gz), 2);
 Serial.print(", ");
}
//
void printAccel() //in ax,ay,az
{
  Serial.print(acc_calibrated[0],2);
  Serial.print(acc_calibrated[1],2);
  Serial.print(acc_calibrated[2],2);
}  
//  // Now we can use the ax, ay, and az variables as we please.
//  // Either print them as raw ADC values, or calculated in g's.
//  Serial.print(imu.calcAccel(imu.ax), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcAccel(imu.ay), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcAccel(imu.az), 2);
//  Serial.print(", ");
//
//}
//
void printMag() //mx,my,mz
{
  Serial.print(mag_calibrated[0],2);
  Serial.print(mag_calibrated[1],2);
  Serial.print(mag_calibrated[2],2);
}
//  // Now we can use the mx, my, and mz variables as we please.
//  // Either print them as raw ADC values, or calculated in Gauss.
//
//  Serial.print(imu.calcMag(imu.mx), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcMag(imu.my), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcMag(imu.mz), 2);
//
//}
