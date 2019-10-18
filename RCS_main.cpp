//Make sure you have the Adafruit_LIS3DH library from https://github.com/adafruit/Adafruit_LIS3DH
//Also have the adafruit_sensor library https://learn.adafruit.com/adafruit-lis3dh-triple-axis-accelerometer-breakout/arduino
//Pressure sensor/ altitude sensor library: https://github.com/sparkfun/SparkFun_MPL3115A2_Breakout_Arduino_Library/archive/master.zip
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"
#include "SparkFunMPL3115A2.h"
#define START_CONTROL_ALTITUDE 0 //Measured in meters above sea level.
//Time variables used to control openlog
unsigned long time1 = millis();
unsigned long time2 = millis();

//important variables
uint8_t  temperature = 0; //The temperature sensor goes in increments of 1 degree celcius. 8 bit number
double angularVelocity = 0.0; //We are making the assumption that
void updateAngularVelocity(); //Function where all the complicated math will be stored to convert the acceleration into rotational velocity
float altitude; //Is in meters currently.

//Custom sensor objects:
Adafruit_LIS3DH lis1 = Adafruit_LIS3DH();
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH();
MPL3115A2 pressureSensor = MPL3115A2();
void setup() {
  //Set up deltatime
  time1 = millis();
  time2 = millis();
  //Set up accelerometer:
  Serial.begin(9600);
  if (! lis1.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  if (! lis2.begin(0x19)) {
    Serial.println("Couldnt start");
    while (1);
  }
  lis1.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  lis2.setRange(LIS3DH_RANGE_4_G);
  //Set up pressure sensor:
  pressureSensor.begin(); //get sensor online
  pressureSensor.setModeAltimeter();
  pressureSensor.setOversampleRate(7); //Set oversample to the reccomended 128
  pressureSensor.enableEventFlags(); // Enable all three pressure and temp event flags
  //Set up I2C Communication
  Wire.begin();
  //Make header for csv file on openlog
  Serial.println("Time, altitude, temperature, angular velocity");
}

void loop() {
  //update time1
  time1 = millis();
  //Update variables (altitude, temperature, pressure, etc.) Velocity must be updated ASAP, because velocity must be calculated from the acceleration, which will be lots of estimates, and could be an issue.
  //Read acceleration.
  lis1.read(); //Acceleration is measured in m/s^2.
  lis2.read();
  //Read Temperature using TC74A0-3.3VAT: (In future, might want to use MPL311A2 to get temperature. It is more accurate and easier to use.)
  Wire.beginTransmission(0x4D); //The address of the temperature sensor is 1001 101  (0x4D)
  Wire.write(0x00);
  Wire.requestFrom(0x4D, 1);
  if (Wire.available()) {
    temperature = Wire.read();
  }
  Wire.endTransmission();
  //Read Altitude:
  altitude = pressureSensor.readAltitude();
  //Every tenth of a second write the sensor data to the SD card.
  if (time1-time2>=100) { //delta time .1 seconds
    time2=millis();
    //write all of the data;
    String dataEntry = String(millis()) + "," + String(altitude) + "," + String(temperature) + "," + String(angularVelocity);
    Serial.println(dataEntry);
   
    }
  //Once the altitude is above START_CONTROL_ALTITUDE, start the auto adjustment algorithm.
  if (altitude > START_CONTROL_ALTITUDE) {
    //Adjustment algorithm

  }
}
void updateAngularVelocity() {
  
}
