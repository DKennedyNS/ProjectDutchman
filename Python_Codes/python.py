import serial
import RPi.GPIO as GPIO
#import time
import re
from library.flightControls import FlightControls

def parseArduino(indexOfVariable, gpsInfo):
    numRegex = "\d|\.|-"

    indexOfNewLine = gpsInfo.find("\r")
    requiredLine = gpsInfo[indexOfVariable:indexOfNewLine]
    requiredData = re.findall(numRegex, requiredLine)

    dataToReturn = ""
    for value in requiredData:
        dataToReturn += value
    
    return dataToReturn
    


def readArduino():
    ser = serial.Serial("/dev/ttyACM0", 9600)
    
    gpsArray = ["-1", "-1", "-1", "-1", "-1", "-1"]
    fileTxt = open("gpsDataFromArduino.txt","a")

    gpsArrayInitialized = False
    while not gpsArrayInitialized:
        count = 0
        
        for data in gpsArray:
            if data != "-1":
                count = count + 1
            else:
                break

        if count >= 5:
            gpsArrayInitialized = True
            break
        
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

        if len(readSerial) > 0:
            if indOfLat >= 0:
                gpsArray[0] = parseArduino(indOfLat, plainGPS)
            elif indOfLong >= 0:
                gpsArray[1] = parseArduino(indOfLong, plainGPS)
            elif indOfIntendedHeading >= 0:
                gpsArray[2] = parseArduino(indOfIntendedHeading, plainGPS)
            elif indOfHeading >= 0 and indOfIntendedHeading < 0:
                gpsArray[3] = parseArduino(indOfHeading, plainGPS)
            elif indOfDistance >= 0:
                gpsArray[4] = parseArduino(indOfDistance, plainGPS)
            elif indOfAltitude >= 0:
                gpsArray[5] = parseArduino(indOfAltitude, plainGPS)

    fileTxt.close()

    return gpsArray


def main():
    #ser = serial.Serial('/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_7543535303835151D012-if00',9600)
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18,GPIO.OUT, initial= GPIO.LOW)

    # [0] = Latitude
    # [1] = Longitude
    # [2] = Intended Heading
    # [3] = Current Heading
    # [4] = Distance
    # [5] = Altitude
    
    #gpsArray = readArduino()
    #for value in gpsArray:
    #    print(value)
    
    glider = FlightControls()
    print(glider.aileronAngle)
    intHeading = 180.545135
    currHeading = 20
    
    #glider.setAngle(30.54584453293809)
    
    while currHeading <= 360:    
        glider.buildPID(currHeading, intHeading)
        glider.updatePID(currHeading)
        
        print("AAngle: ",glider.aileronAngle)
        
        currHeading += 10
    

if __name__ == "__main__":
    main()