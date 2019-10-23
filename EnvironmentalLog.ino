/******************************************************************************
  BME280Compensated.ino
  Marshall Taylor @ SparkFun Electronics
  April 4, 2017
  https://github.com/sparkfun/CCS811_Air_Quality_Breakout
  https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library
  This example uses a BME280 to gather environmental data that is then used
  to compensate the CCS811.
  Hardware Connections (Breakoutboard to Arduino):
  3.3V to 3.3V pin
  GND to GND pin
  SDA to A4
  SCL to A5
  Resources:
  Uses Wire.h for i2c operation
  Development environment specifics:
  Arduino IDE 1.8.1
  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/

/************************** Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include "config.h"

/************************ Example Starts Here *******************************/

#include <Wire.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
OpenLog myLog; //Create instance

const byte OpenLogAddress = 42; //Default Qwiic OpenLog I2C address

#include <SparkFunBME280.h>
#include <SparkFunCCS811.h>

#define CCS811_ADDR 0x5B //Default I2C Address
//#define CCS811_ADDR 0x5A //Alternate I2C Address

//Global sensor objects
CCS811 myCCS811(CCS811_ADDR);
BME280 myBME280;

// set up the 'temperature' feed
AdafruitIO_Feed *temperatureIO = io.feed("temperature");
AdafruitIO_Feed *humidityIO = io.feed("humidity");
AdafruitIO_Feed *pressureIO = io.feed("pressure");
AdafruitIO_Feed *altitudeIO = io.feed("altitude");
AdafruitIO_Feed *eco2IO = io.feed("eco2");
AdafruitIO_Feed *tvocIO = io.feed("tvoc");

float temp;
float humidity;
float pressure;
float alt;
float eco2;
float tvoc;

const int STATUS_LED = 13;
//This is in milliseconds
const int SAMPLE_RATE = 15000;

void setup()
{
  pinMode(STATUS_LED, OUTPUT);

  Serial.begin(115200);

  Serial.println();
  Serial.println("Apply BME280 data to CCS811 for compensation.");

  Wire.begin();//initialize I2C bus
  myLog.begin(); //Open connection to OpenLog (no pun intended)

  //This begins the CCS811 sensor and prints error status of .begin()
  CCS811Core::status returnCode = myCCS811.begin();
  if (returnCode != CCS811Core::SENSOR_SUCCESS)
  {
    Serial.println("Problem with CCS811");
    printDriverError(returnCode);
  }
  else
  {
    Serial.println("CCS811 online");
  }


  //Initialize BME280
  //For I2C, enable the following and disable the SPI section
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;
  myBME280.settings.runMode = 3; //Normal mode
  myBME280.settings.tStandby = 0;
  myBME280.settings.filter = 4;
  myBME280.settings.tempOverSample = 5;
  myBME280.settings.pressOverSample = 5;
  myBME280.settings.humidOverSample = 5;

  //Calling .begin() causes the settings to be loaded
  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  byte id = myBME280.begin(); //Returns ID of 0x60 if successful
  if (id != 0x60)
  {
    Serial.println("Problem with BME280");
  }
  else
  {
    Serial.println("BME280 online");
  }

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

}
//---------------------------------------------------------------
void loop()
{
  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  //Check to see if data is available
  if (myCCS811.dataAvailable())
  {
    //Calling this function updates the global tVOC and eCO2 variables
    myCCS811.readAlgorithmResults();
    //printData fetches the values of tVOC and eCO2
    printData();
    uploadData();
    recordData();

    float BMEtempC = myBME280.readTempC();
    float BMEhumid = myBME280.readFloatHumidity();

    Serial.print("Applying new values (deg C, %): ");
    Serial.print(BMEtempC);
    Serial.print(",");
    Serial.println(BMEhumid);
    Serial.println();

    //This sends the temperature data to the CCS811
    myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);
  }
  else if (myCCS811.checkForStatusError())
  {
    Serial.println(myCCS811.getErrorRegister()); //Prints whatever CSS811 error flags are detected
  }

  digitalWrite(STATUS_LED, HIGH);
  delay(250);
  digitalWrite(STATUS_LED, LOW);

  delay(SAMPLE_RATE); //Wait for next reading
}

//---------------------------------------------------------------
void recordData()
{
  myLog.append("EnvironData.csv");

  String toWrite;
  toWrite.concat(String(temp));
  toWrite.concat("\t");
  toWrite.concat(String(humidity));
  toWrite.concat("\t");
  toWrite.concat(String(pressure));
  toWrite.concat("\t");
  toWrite.concat(String(alt));
  toWrite.concat("\t");
  toWrite.concat(String(eco2));
  toWrite.concat("\t");
  toWrite.concat(String(tvoc));

  myLog.println(toWrite);
}

void uploadData()
{
  temperatureIO->save(temp);
  humidityIO->save(humidity);
  pressureIO->save(pressure);
  altitudeIO->save(alt);
  eco2IO->save(eco2);
  tvocIO->save(tvoc);
}

void printData()
{
  Serial.print(" CO2[");
  eco2 = myCCS811.getCO2();
  Serial.print(eco2);
  Serial.print("]ppm");

  Serial.print(" TVOC[");
  tvoc = myCCS811.getTVOC();
  Serial.print(tvoc);
  Serial.print("]ppb");

  Serial.print(" temp[");
  temp = myBME280.readTempC();
  Serial.print(temp, 1);
  Serial.print("]C");

  //Serial.print(" temp[");
  //Serial.print(myBME280.readTempF(), 1);
  //Serial.print("]F");

  Serial.print(" pressure[");
  pressure = myBME280.readFloatPressure();
  Serial.print(pressure, 2);
  Serial.print("]Pa");

  //Serial.print(" pressure[");
  //Serial.print((myBME280.readFloatPressure() * 0.0002953), 2);
  //Serial.print("]InHg");

  Serial.print(" altitude[");
  alt = myBME280.readFloatAltitudeMeters();
  Serial.print(alt, 2);
  Serial.print("]m");

  //Serial.print("altitude[");
  //Serial.print(myBME280.readFloatAltitudeFeet(), 2);
  //Serial.print("]ft");

  Serial.print(" humidity[");
  humidity = myBME280.readFloatHumidity();
  Serial.print(humidity, 0);
  Serial.print("]%");

  Serial.println();
}

//printDriverError decodes the CCS811Core::status type and prints the
//type of error to the serial terminal.
//
//Save the return value of any function of type CCS811Core::status, then pass
//to this function to see what the output was.
void printDriverError( CCS811Core::status errorCode )
{
  switch ( errorCode )
  {
    case CCS811Core::SENSOR_SUCCESS:
      Serial.print("SUCCESS");
      break;
    case CCS811Core::SENSOR_ID_ERROR:
      Serial.print("ID_ERROR");
      break;
    case CCS811Core::SENSOR_I2C_ERROR:
      Serial.print("I2C_ERROR");
      break;
    case CCS811Core::SENSOR_INTERNAL_ERROR:
      Serial.print("INTERNAL_ERROR");
      break;
    case CCS811Core::SENSOR_GENERIC_ERROR:
      Serial.print("GENERIC_ERROR");
      break;
    default:
      Serial.print("Unspecified error.");
  }
}
