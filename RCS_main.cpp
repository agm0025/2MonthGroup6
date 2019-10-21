//Make sure you have the Adafruit_LIS3DH library from https://github.com/adafruit/Adafruit_LIS3DH
//Also have the adafruit_sensor library https://learn.adafruit.com/adafruit-lis3dh-triple-axis-accelerometer-breakout/arduino
//Pressure sensor/ altitude sensor library: https://github.com/adafruit/Adafruit_BMP3XX
//Openlog libraries: https://github.com/sparkfun/OpenLog/tree/master/Libraries
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"
#include "Adafruit_BMP3XX.h"
#include <SoftwareSerial.h>
#define START_CONTROL_ALTITUDE 0 //Measured in meters above sea level.
#define SEALEVELPRESSURE_HPA (1013.25) //Pressure at sea level
#define AXIS x //Which orientation is the LIS3DH in? IE which axis is = to circular acceleration.
#define ACCELEROMETER_RADIUS 5 //in meters
//Time variables used to control openlog
unsigned long time1 = millis();
unsigned long time2 = millis();
//important variables
uint8_t  temperature = 0; //The temperature sensor goes in increments of 1 degree celcius. 8 bit number
double angularVelocity = 0.0; //We are making the assumption that
void updateAngularVelocity(); //Function where all the complicated math will be stored to convert the acceleration into rotational velocity
float altitude; //Is in meters currently.

//Custom sensor objects:
//Custom objects:
Adafruit_LIS3DH lis1 = Adafruit_LIS3DH();
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH();
Adafruit_BMP3XX pressureSensor;
SoftwareSerial OpenLog(0, 5);
void setup() {
  //Set up openlog
  OpenLog.begin(9600);
  //Set up deltatime
  time1 = millis();
  time2 = millis();
  //Set up accelerometer:
  Serial.begin(9600);
  if (! lis1.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    OpenLog.println("Couldnt start");
    while (1);
  }
  if (! lis2.begin(0x19)) {
    Serial.println("Couldnt start");
    OpenLog.println("Couldnt start");
    while (1);
  }
  lis1.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  lis2.setRange(LIS3DH_RANGE_4_G);
  //Set up pressure sensor:
  if (!pressureSensor.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    OpenLog.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  pressureSensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  pressureSensor.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  pressureSensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //Set up I2C Communication
  Wire.begin();
  //Make header for csv file on openlog
  Serial.println("Time, altitude, temperature, angular velocity");
  OpenLog.println("Time, altitude, temperature, angular velocity");
}

void loop() {
  //update time1
  time1 = millis();
  //Update variables (altitude, temperature, pressure, etc.) Velocity must be updated ASAP, because velocity must be calculated from the acceleration, which will be lots of estimates, and could be an issue.
  //Read acceleration.
  lis1.read(); //Acceleration is measured in m/s^2.
  lis2.read();
  //Read Altitude:
  pressureSensor.performReading();
  altitude = pressureSensor.readAltitude(SEALEVELPRESSURE_HPA); //In meters
  //Read temperature:
  temperature = pressureSensor.temperature;
  //Every tenth of a second write the sensor data to the SD card.
  if (time1-time2>=100) { //delta time .1 seconds
    time2=millis();
    //write all of the data;
    String dataEntry = String(millis()) + "," + String(altitude) + "," + String(temperature) + "," + String(angularVelocity);
    Serial.println(dataEntry);
    OpenLog.println(dataEntry);

    }
  //Once the altitude is above START_CONTROL_ALTITUDE, start the auto adjustment algorithm.
  if (altitude > START_CONTROL_ALTITUDE) {
    //Adjustment algorithm
  }
}
void updateAngularVelocity() {
  angularVelocity = sqrt(((lis1.AXIS + lis2.AXIS)/2)*ACCELEROMETER_RADIUS);
}
