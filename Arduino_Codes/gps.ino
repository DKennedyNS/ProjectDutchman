#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

int RXPin = 6;
int TXPin = 7;
int GPSBaud = 9600;

const float destLat = 44.760259;
const float destLong = -63.657050;

float lastLat = 0;
float lastLong = 0;

TinyGPSPlus gps;
SoftwareSerial mySerial(RXPin,TXPin);


void setup() {

  
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
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

        destHeading = gps.courseTo(latitude,longitude, destLat, destLong);
        distance = gps.distanceBetween(latitude,longitude, destLat, destLong);
        

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
    else{
      //Serial.println(F("-"));
    }
  }
}
