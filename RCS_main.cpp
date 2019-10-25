//Make sure you have the Adafruit_LIS3DH library from https://github.com/adafruit/Adafruit_LIS3DH
//Also have the adafruit_sensor library https://learn.adafruit.com/adafruit-lis3dh-triple-axis-accelerometer-breakout/arduino
//Pressure sensor/ altitude sensor library: https://github.com/adafruit/Adafruit_BMP3XX
//Openlog libraries: https://github.com/sparkfun/OpenLog/tree/master/Libraries
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"
#include "Adafruit_BMP3XX.h"
#include <SoftwareSerial.h>
#include <SPI.h>
#define START_CONTROL_ALTITUDE 0 //Measured in meters above sea level.
#define AXIS z//Which orientation is the LIS3DH in? IE which axis is = to circular acceleration.
#define AXIS_TAN x 
#define START_CONTROL_ALTITUDE 0 //Measured in meters above sea level.
#define SEALEVELPRESSURE_HPA (1013.25) //Pressure at sea level
#define ACCELEROMETER_RADIUS 4//0.1074 //in meters
// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10
//Time variables used to control openlog
//Soelnoid valve pins
#define VALVE_CLOCKWISE 4
#define VALVE_COUNTERCLOCKWISE 7
unsigned long time1 = millis();
unsigned long time2 = millis();
unsigned long timer = millis();
uint8_t whichDirection = 0; // 1 = clockwise and 0 = counterclockwise
//important variables
uint8_t  temperature = 0; //The temperature sensor goes in increments of 1 degree celcius. 8 bit number
double angularVelocity = 0.0; //
double velocityDirection = 0.0;
float altitude; //Is in meters currently.

//Custom objects:
Adafruit_LIS3DH lis1 = Adafruit_LIS3DH();
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
Adafruit_BMP3XX pressureSensor;
SoftwareSerial OpenLog(0, 5);
void setup() {
  pinMode(VALVE_CLOCKWISE, OUTPUT);
  pinMode(VALVE_COUNTERCLOCKWISE, OUTPUT);
  //Set up openlog
  OpenLog.begin(9600);
  //Set up deltatime
  time1 = millis();
  time2 = millis();
  //Set up accelerometer:
  Serial.begin(9600);
  if (! lis1.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    OpenLog.println("Couldnt start");
    Serial.println("Couldnt start lis1");
    while (1);
  } else {
    Serial.println("started lis1");
    }
  if (! lis2.begin(0x18)) {
    OpenLog.println("Couldnt start");
    Serial.println("Couldnt start lis2");
    while (1);
  } else {
    Serial.println("started lis2");
    }
  lis1.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!
  lis2.setRange(LIS3DH_RANGE_16_G);
  //Set up pressure sensor:
  if (!pressureSensor.begin()) {
    OpenLog.println("Could not find a valid BMP3 sensor, check wiring!");
    Serial.println("Could not find pressure sensor!"); 
    //while (1);
  } else {
     Serial.println("Found pressure sensor!");
    }
  pressureSensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  pressureSensor.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  pressureSensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //Set up I2C Communication
  Wire.begin();
  //Make header for csv file on openlog
  OpenLog.println("Time, altitude, temperature, angular velocity");
}

void loop() {
  //Update variables (altitude, temperature, pressure, etc.) Velocity must be updated ASAP, because velocity must be calculated from the acceleration, which will be lots of estimates, and could be an issue.
  
  //Read acceleration.
  sensors_event_t accelEvent1; 
  lis1.getEvent(&accelEvent1);
  sensors_event_t accelEvent2; 
  lis2.getEvent(&accelEvent2);
  angularVelocity = sqrt((abs(accelEvent1.acceleration.AXIS + accelEvent2.acceleration.AXIS)/2)*ACCELEROMETER_RADIUS);
  //Read Altitude:
  pressureSensor.performReading();
  altitude = pressureSensor.readAltitude(SEALEVELPRESSURE_HPA); //In meters
  //Read temperature:
  temperature = pressureSensor.temperature;
  //Every tenth of a second write the sensor data to the SD card.
  if (millis()-time2>=100) { //delta time .1 seconds
    time2=millis();
    //write all of the data;
    //String dataEntry = String(millis()-time1) + ", alt: " + String(altitude) + ", temp: " + String(temperature) + ", lis1z: " + String(accelEvent1.acceleration.z) + " lis2z: "+ String(accelEvent2.acceleration.z) + " angAcl: " + angularVelocity +", direction: " + velocityDirection;
    //OpenLog.println(dataEntry);
    //Serial.println(dataEntry);
    }
  //Once the altitude is above START_CONTROL_ALTITUDE, start the auto adjustment algorithm.
  if (true/*abs(accelEvent1.acceleration.z)>1.0/*START_CONTROL_ALTITUDE*/) {
    if (angularVelocity > 2 && whichDirection == 0) {
      Serial.println("ROTATING t= " + String(millis()));
      if (velocityDirection > 0) {
          
          whichDirection = 1; //clockwise
          digitalWrite(VALVE_CLOCKWISE, HIGH);
          Serial.println("It is rotating counterclockwise. Activating clockwise booster");
        } else {
          whichDirection = 2; //counterclockwise
          digitalWrite(VALVE_COUNTERCLOCKWISE, HIGH);
          Serial.println("It is rotating clockwise. Activating counterclockwise booster");
        }
        timer = millis() + 1000;
    }
  } else {
    pinMode(VALVE_CLOCKWISE, LOW);
    }
  if (whichDirection != 0 && timer < millis()) {
      whichDirection = 0;
      digitalWrite(VALVE_CLOCKWISE, LOW);
      digitalWrite(VALVE_COUNTERCLOCKWISE, LOW);
      Serial.println("STOPPED ROTATION. timer = " + String(timer) + " time: " + String(millis()));
    }
  //Update velocity direction
  velocityDirection = (millis()-time1)*(accelEvent1.acceleration.AXIS_TAN+accelEvent2.acceleration.AXIS_TAN)*(0.5);
    //update time1
  time1 = millis();
}
