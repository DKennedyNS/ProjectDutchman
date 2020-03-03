import serial
import RPi.GPIO as GPIO
import time
import re

def parseArduino(gpsInfo, indexOfVariable){
    numRegex = "\d|\."

    indexOfNewLine = gpsInfo.find("\r")
    requiredLine = gpsInfo[indexOfVariable:indexOfNewLine]
    requiredData = re.findall(numRegex, requiredLine)

    dataToReturn = ""
    for value in requiredData:
        dataToReturn += value
    
    return dataToReturn
}

#ser = serial.Serial('/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_7543535303835151D012-if00',9600)
ser=serial.Serial("/dev/ttyACM0", 9600)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18,GPIO.OUT, initial= GPIO.LOW)

latitude = "0"
longitude = "0"
intendedHeading = "0"
heading = "0"
distance = "0"
altitude = "0"

fileTxt = open("gpsDataFromArduino.txt","a")

while True:
    readSerial=ser.readline()

    plainGPS = str(readSerial)    
    
    #!!! indOfX are checks for what the Arduino is currently sending !!!
    # if the variable equals anything other than -1, that's the information currently being read
    indOfLat = plainGPS.find('Lat')
    indOfLong = plainGPS.find('Long')
    indOfIntendedHeading = plainGPS.find('Intended')
    indOfHeading = plainGPS.find('Heading')
    indOfDistance = plainGPS.find('Distance')
    indOfAltitude = plainGPS.find('Altitude')

    if len(readSerial) > 1:
        if indOfLat >= 0:
            latitude = parseArduino(indOfLat, plainGPS)
        elif indOfLong >= 0:
            longitude = parseArduino(indOfLong, plainGPS)
        elif indOfIntendedHeading >= 0:
            intendedHeading = parseArduino(indOfIntendedHeading, plainGPS)
        elif indOfHeading >= 0 and indOfIntendedHeading < 0:
            heading = parseArduino(indOfHeading, plainGPS)
        elif indOfDistance >= 0:
            distance = parseArduino(indOfDistance, plainGPS)
        elif indOfAltitude >= 0:
            altitude = parseArduino(indOfAltitude, plainGPS)


# OLD CODE
#indOfNewLine = plainGPS.find("\r")
            #latitude = plainGPS[indOfLat:indOfNewLine]
            #latNum = re.findall(numRegex, latitude)
            #latitude = ""
            #for ele in latNum:
            #    latitude += ele
            #print("Latitude: ", latitude)
            #fileTxt.write("Latitude: ", latitude)
            #fileTxt.write('\n')

            # indOfNewLine = plainGPS.find("\r")
            # longitude = plainGPS[indOfLong:indOfNewLine]
            # longNum = re.findall(numRegex, longitude)
            # longitude = ""
            # for ele in longNum:
            #     longitude += ele
            # print("Longitude: ", longitude)
            # fileTxt.write("Longitude: ", longitude)
            # fileTxt.write('\n')

            # indOfNewLine = plainGPS.find("\r")
            # intendedHeading = plainGPS[indOfIntendedHeading:indOfNewLine]
            # intendedNum = re.findall(numRegex, intendedHeading)
            # intendedHeading = ""
            # for ele in intendedNum:
            #     intendedHeading += ele
            # print("Intended Heading: ", intendedHeading);
            # fileTxt.write("Intended Heading: ", intendedHeading);
            # fileTxt.write('\n')

            # indOfNewLine = plainGPS.find("\r")
            # heading = plainGPS[indOfHeading:indOfNewLine]
            # headingNum = re.findall(numRegex, heading)
            # heading = ""
            # for ele in headingNum:
            #     heading += ele
            # print("Current Heading: ", heading)
            # fileTxt.write("Current Heading: ", heading)
            # fileTxt.write('\n')

            # indOfNewLine = plainGPS.find("\r");
            # distance = plainGPS[indOfDistance:indOfNewLine];
            # distanceNum = re.findall(numRegex, distance);
            # distance = "";
            # for ele in distanceNum:
            #     distance += ele;
            # print("Distance: ", distance);
            # fileTxt.write("Distance: ", distance);
            # fileTxt.write('\n')

            # indOfNewLine = plainGPS.find("\r");
            # altitude = plainGPS[indOfAltitude:indOfNewLine];
            # altitudeNum = re.findall(numRegex, altitude);
            # altitude = "";
            # for ele in altitudeNum:
            #     altitude += ele;
            # print("Altitude: ", altitude);
            # fileTxt.write("Altitude: ", altitude);
            # fileTxt.write('\n')

#if(checkBadData >= 0):
    #    GPIO.output(18,GPIO.LOW)
    #else:
    #    GPIO.output(18,GPIO.HIGH)
    #    print("Longitude: ", longitude)