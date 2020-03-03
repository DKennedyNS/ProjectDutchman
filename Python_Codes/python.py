import serial
import RPi.GPIO as GPIO
import time
import re

#ser = serial.Serial('/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_7543535303835151D012-if00',9600)
ser=serial.Serial("/dev/ttyACM0", 9600)
GPIO.setwarnings(False);
GPIO.setmode(GPIO.BCM)
GPIO.setup(18,GPIO.OUT, initial= GPIO.LOW)

numRegex = "\d|\."

while True:
    readSerial=ser.readline()
    
    arData = readSerial.split()
    #print(arData)
    
    latitude = "0"
    longitude = "0"
    checkBadData = -1;
    
    dataBegin = str(arData[0])
    strEnd = ""
    
    fileTxt = open("gpsDataFromArduino.txt","a")
    
    plainGPS = str(readSerial)    
    
    #if(indOfNewLine >= 0){
    #fileTxt.write('\n');
    #}
    #fileTxt.write(plainGPS)
    
    
    indOfLat = dataBegin.find('Lat')
    indOfLong = dataBegin.find('Long')
    indOfIntendedHeading = dataBegin.find('Intended')
    indOfHeading = dataBegin.find('Heading')
    indOfDistance = dataBegin.find('Distance')
    indOfAltitude = dataBegin.find('Altitude')
    if len(arData) > 1:
        if indOfLat >= 0:
            indOfNewLine = plainGPS.find("\r")
            latitude = plainGPS[indOfLat:indOfNewLine];
            latNum = re.findall(numRegex, latitude);
            
            latitude = "";
            for ele in latNum:
                latitude += ele;
            print("Latitude: ", latitude)
            fileTxt.write("Latitude: ", latitude)
            fileTxt.write('\n')
        elif indOfLong >= 0:
            indOfNewLine = plainGPS.find("\r")
            longitude = plainGPS[indOfLong:indOfNewLine];
            longNum = re.findall(numRegex, longitude);
            
            longitude = "";
            for ele in longNum:
                longitude += ele;
            print("Longitude: ", longitude)
            fileTxt.write("Longitude: ", longitude)
            fileTxt.write('\n')
        elif indOfIntendedHeading >= 0:
            indOfNewLine = plainGPS.find("\r");
            intendedHeading = plainGPS[indOfIntendedHeading:indOfNewLine];
            intendedNum = re.findall(numRegex, intendedHeading);
            
            intendedHeading = "";
            for ele in intendedNum:
                intendedHeading += ele;
            
            print("Intended Heading: ", intendedHeading);
            fileTxt.write("Intended Heading: ", intendedHeading);
            fileTxt.write('\n')
        elif indOfHeading >= 0 and indOfIntendedHeading < 0:
            indOfNewLine = plainGPS.find("\r");
            heading = plainGPS[indOfHeading:indOfNewLine];
            headingNum = re.findall(numRegex, heading);
            
            heading = "";
            for ele in headingNum:
                heading += ele;
                
            print("Current Heading: ", heading);
            fileTxt.write("Current Heading: ", heading);
            fileTxt.write('\n')
        elif indOfDistance >= 0:
            indOfNewLine = plainGPS.find("\r");
            distance = plainGPS[indOfDistance:indOfNewLine];
            distanceNum = re.findall(numRegex, distance);
            
            distance = "";
            for ele in distanceNum:
                distance += ele;
                
            print("Distance: ", distance);
            fileTxt.write("Distance: ", distance);
            fileTxt.write('\n')
        elif indOfAltitude >= 0:
            indOfNewLine = plainGPS.find("\r");
            altitude = plainGPS[indOfAltitude:indOfNewLine];
            altitudeNum = re.findall(numRegex, altitude);
            
            altitude = "";
            for ele in altitudeNum:
                altitude += ele;
                
            print("Altitude: ", altitude);
            fileTxt.write("Altitude: ", altitude);
            fileTxt.write('\n')
            #if(checkBadData >= 0):
            #    GPIO.output(18,GPIO.LOW)
            #else:
            #    GPIO.output(18,GPIO.HIGH)
            #    print("Longitude: ", longitude)
    
