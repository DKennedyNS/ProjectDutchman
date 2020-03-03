#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(6,7);

void setup() {

  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  char temp = 0;
    while(mySerial.available() > 0)
    {
      temp = mySerial.read();
      Serial.print(temp);
    }
}
