//Make sure you have the Adafruit_LIS3DH library from https://github.com/adafruit/Adafruit_LIS3DH
//Also have the adafruit_sensor library https://learn.adafruit.com/adafruit-lis3dh-triple-axis-accelerometer-breakout/arduino
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#define START_CONTROL_ALTITUDE = 0; //Measured in meters above sea level.

unsigned long time = millis();
double altitude = 0;
double temperature = 0;
double pressure = 0;
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
void setup() {
  //Set up accelerometer
  Serial.begin(9600);
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
}

void loop() {
  //Update variables (altitude, temperature, pressure, etc.)
  
  //Every tenth of a second write the sensor data to the SD card.

  //Once the altitude is above START_CONTROL_ALTITUDE, start the auto adjustment algorithm.

    //See if acceleration around verticle axis 
}
