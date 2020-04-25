// # Editor     : Roy from DFRobot
// # Date       : 10.12.2013
// # Product name: 6 Dof shield for Arduino
// # Product SKU : DFR0209
// # Version     : 0.1
// # Description:
// # The sketch for driving the 6 Dof shield for Arduino via I2C interface

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#include <Wire.h>

// ACCELEROMETER VARIABLES
int16_t angle[2]; // pitch & roll

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
int rawSixDof[6];
// ----------------------------

// GPS VARIABLES
int RXPin = 6;
int TXPin = 7;
int GPSBaud = 9600;

const float destLat = 44.760259;
const float destLong = -63.657050;

const float currLat = 44.642344;
const float currLong = -63.590759;

float lastLat = 0;
float lastLong = 0;

TinyGPSPlus gps;
SoftwareSerial mySerial(RXPin,TXPin);
// ------------------------------

void setup()
{
  Serial.begin(9600);
  // ACCEL..
  Wire.begin();

  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);

  // GPS...
  mySerial.begin(9600);
}

void loop() {
  //Serial.println("ACCELEROMETER STUFF");
  logAccelerometer();
  //Serial.println("GPS STUFF");
  logGPS();
  //Serial.println("END");
  delay(500);
  
}

void logGPS(){
  while(mySerial.available() > 0)
  {
    if(gps.encode(mySerial.read()))
    {
      if(gps.location.isValid())
      {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();

        float destHeading = 0;
        float heading = 0;
        float distance = 0;
        int numSatellites = 0;

        //destHeading = gps.courseTo(latitude,longitude, destLat, destLong);
        //distance = gps.distanceBetween(latitude,longitude, destLat, destLong);

        destHeading = gps.courseTo(currLat,currLong, destLat, destLong);
        distance = gps.distanceBetween(currLat,currLong, destLat, destLong);
        
        float alt = gps.altitude.value();
        Serial.print("Lat "); Serial.println(latitude,6); 
        Serial.print("Long "); Serial.println(longitude,6);
        Serial.print("Distance "); Serial.println(distance,6);
        Serial.print("Intended Heading "); Serial.println(destHeading,6);
        Serial.print("Altitude "); Serial.println(alt);

        if(lastLong != 0){
          heading = gps.courseTo(latitude,longitude, lastLat, lastLong);
        }

        lastLat = latitude;
        lastLong = longitude;

        Serial.print("Heading "); Serial.println(heading,6);
      } 
    }
  }
  Serial.println("GPS CHECKED");
}

void logAccelerometer(){
  sixDOF.getRawValues(rawSixDof);
  for (int i=0; i<6; i++)              //output the raw data
  {
    switch (i)
    {
        case 0:
        //Serial.print("AccX ");
        break;
      case 1:
        //Serial.print("AccY ");
        break;
      case 2:
        //Serial.print("AccZ ");
        break;
      case 3:
        //Serial.print("gyroX ");
        break;
      case 4:
        //Serial.print("gyroY ");
        break;
      case 5:
        //Serial.print("gyroZ ");
      break;
        default:
            Serial.print("Err");
    }
    //Serial.println(rawSixDof[i]);
  }
  Serial.println("");

  float destHeading = 0;
  float distance = 0;

  destHeading = gps.courseTo(currLat,currLong, destLat, destLong);
  distance = gps.distanceBetween(currLat,currLong, destLat, destLong);

  Serial.print("Distance "); Serial.println(distance,6);
  Serial.print("Intended Heading "); Serial.println(destHeading,6);

  
  angle[0] = _atan2(rawSixDof[0],rawSixDof[2]);
  angle[1] = _atan2(rawSixDof[1],rawSixDof[2]);

  Serial.print("ROLL: ");              //pitch & roll
  Serial.println(angle[0]/10.0);
  Serial.print("PITCH: ");
  Serial.println(angle[1]/10.0);
  Serial.println("");
}

int16_t _atan2(int32_t y, int32_t x)   //get the _atan2
{
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) )
  {
    a = 573 * z / (1.0f + 0.28f * z * z);
    if (x<0)
    {
    if (y<0) a -= 1800;
    else a += 1800;
    }
  }
  else
  {
    a = 900 - 573 * z / (z * z + 0.28f);
    if (y<0) a -= 1800;
  }
  return a;
}

/*
STRAIGHT 
Acc.x :11 (0)
Acc.y :-24(0)
Acc.z :234(200)
gyro.x :20(0)
gyro.y :13(0)
gyro.z :403(400)

X:2.60
Y:-5.80

RIGHT SIDE
Acc.x :271 (270)
Acc.y :-2 (0)
Acc.z :-19(0)
gyro.x :24(0)
gyro.y :14(0)
gyro.z :394(400)

X:94.00
Y:-174.00

LEFT SIDE
Acc.x :-254 (-270)
Acc.y :-7 (0)
Acc.z :-12(0)
gyro.x :25(0)
gyro.y :17(0)
gyro.z :417(400)

X:-92.80
Y:-149.50

FRONT
Acc.x :35 (0)
Acc.y :-263(-270)
Acc.z :-27(0)
gyro.x :62(45 / 0)
gyro.y :-27(0)
gyro.z :408(400)

X:127.80
Y:-95.90

BACK
Acc.x :52 (0 / 45)
Acc.y :254(270)
Acc.z :-14(0)
gyro.x :80(90 / 45 / 0)
gyro.y :13(0)
gyro.z :403(400)

X:105.10
Y:93.10

Only rawSixDof array's 0,1,2 indices are useful. The rest can be deleted.
However, they have to be deleted in the library.

BigX > 0 means roll right
BigX < 0 means roll left
BigY > 0 means pitch back
BigY < 0 means pitch forwards

*/
