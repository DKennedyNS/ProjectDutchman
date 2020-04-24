import serial
import RPi.GPIO as GPIO
#import time
import re
from library.flightControls import FlightControls
from time import sleep

ser = serial.Serial("/dev/ttyACM0", 9600)

def parseArduino(indexOfVariable, gpsInfo):
    numRegex = "\d|\.|[-]"

    indexOfNewLine = gpsInfo.find("\r")
    requiredLine = gpsInfo[indexOfVariable:indexOfNewLine]
    requiredData = re.findall(numRegex, requiredLine)

    dataToReturn = ""
    for value in requiredData:
        dataToReturn += value
    
    return dataToReturn
    


def readArduino():
    
    
    gpsArray = ["0", "0", "0", "0", "0", "0", "-1", "-1"]
    #fileTxt = open("gpsDataFromArduino.txt","a")

    gpsArrayInitialized = False
    while not gpsArrayInitialized:
        count = 0
        
        for data in gpsArray:
            if data != "-1":
                count = count + 1
            else:
                break

        if count > 7:
            gpsArrayInitialized = True
            break
        
        readSerial=ser.readline()
        #readSerial = "PITCH 58";

        plainGPS = str(readSerial)
        #print(plainGPS)
        
        #!!! indOfX are checks for what the Arduino is currently sending !!!
        # if the variable equals anything other than -1, that's the information currently being read
        indOfLat = plainGPS.find('Lat')
        indOfLong = plainGPS.find('Long')
        indOfIntendedHeading = plainGPS.find('Intended')
        indOfHeading = plainGPS.find('Heading')
        indOfDistance = plainGPS.find('Distance')
        indOfAltitude = plainGPS.find('Altitude')
        #the following two may be backwards.
        indOfRoll = plainGPS.find('ROLL')
        indOfPitch = plainGPS.find('PITCH')
        
        termsArray = ['Lat','Long','Intended','Heading','Distance','Altitude','ROLL','PITCH']
        
        indOfX = -1
        
        for i in range(0,8):
            indOfX = plainGPS.find(termsArray[i])
            if indOfX > 0:
                gpsArray[i] = parseArduino(indOfX, plainGPS)
                break
        


        #if len(readSerial) > 0:
        #    if indOfLat >= 0:
        #        gpsArray[0] = parseArduino(indOfLat, plainGPS)
         #   elif indOfLong >= 0:
         #       gpsArray[1] = parseArduino(indOfLong, plainGPS)
         #   elif indOfIntendedHeading >= 0:
        #        gpsArray[2] = parseArduino(indOfIntendedHeading, plainGPS)
        #    elif indOfHeading >= 0 and indOfIntendedHeading < 0:
        #        gpsArray[3] = parseArduino(indOfHeading, plainGPS)
        #    elif indOfDistance >= 0:
        #        gpsArray[4] = parseArduino(indOfDistance, plainGPS)
        #    elif indOfAltitude >= 0:
        #        gpsArray[5] = parseArduino(indOfAltitude, plainGPS)
        #    elif indOfRoll >= 0:
        #        gpsArray[6] = parseArduino(indOfRoll, plainGPS)
        #    elif indOfPitch >= 0:
        #        gpsArray[7] = parseArduino(indOfPitch, plainGPS)
                

    #fileTxt.close()

    return gpsArray


def main():
    #ser = serial.Serial('/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_7543535303835151D012-if00',9600)
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18,GPIO.OUT, initial= GPIO.LOW)
    
    aileronOneUp = 35 #for lift
    aileronOneDown = 35 #for pitch down
    
    aileronTwoUp = -35 #for lift
    aileronTwoDown = 35 #for pitch down
    
   # print("Initializing glider state")
    
    gliderRollRightSense = -20
    gliderRollLeftSense = 20
    gliderPitchUpSense = -20
    gliderPitchDownSense = 20
    
    glider = FlightControls()
    glider.setAileronOne(0)
    sleep(1)
    glider.setAileronTwo(0)
    sleep(1)
    
    
    while True:
        # [0] = Latitude
        # [1] = Longitude
        # [2] = Intended Heading
        # [3] = Current Heading
        # [4] = Distance
        # [5] = Altitude
        # [6] = Roll
        # [7] = Pitch

        gpsArray = readArduino()
        #for value in gpsArray:
        #    print(value)
        #print(gpsArray[6])
        #print(gpsArray[7])

        
        #print(glider.aileronAngle)
        #intHeading = 180.545135
        #currHeading = 20
        
       # glider.setAileronOne(0)
        #glider.setAileronTwo(0)
        
        #glider.setAileronOne(aileronOneUp)
        #glider.setAileronTwo(aileronTwoUp)
        
        
        #accelerateFile = open("ReadAccelerometer.txt", "a")
        #toWrite = "ROLL: " + gpsArray[6] + "\nPITCH: " + gpsArray[7] + "\n"
        #accelerateFile.write(toWrite)
        try:
            #if glider is pitching down, raise ailerons
            if float(gpsArray[7]) > gliderPitchDownSense:
                glider.setAileronOne(aileronOneUp)
                glider.setAileronTwo(aileronTwoUp)
                
                print("Pitching Down")
                
                #accelerateFile.write("Pitching Down\n")
            #if glider is pitching up, lower ailerons
            elif float(gpsArray[7]) < gliderPitchUpSense:
                glider.setAileronOne(aileronOneDown)
                glider.setAileronTwo(aileronTwoDown)
                
                print("Pitching Up")
                
                #accelerateFile.write("Pitching Up\n")
            #if glider is rolling right, raise left aileron
            elif float(gpsArray[6]) < gliderRollRightSense:
                glider.setAileronOne(aileronOneUp)
                glider.setAileronTwo(aileronTwoDown)
                
                print("Rolling Right")
                #print(gpsArray[0])
                #accelerateFile.write("Rolling Right\n")
            #if glider is rolling left, raise right aileron
            elif float(gpsArray[6]) > gliderRollLeftSense:
                glider.setAileronOne(aileronOneDown)
                glider.setAileronTwo(aileronTwoUp)
                
                print("Rolling Left")
                
                #accelerateFile.write("Rolling Left\n")
            else:
            #if glider is in intended state, flatten ailerons.
                glider.setAileronOne(0)
                glider.setAileronTwo(0)
                #print("intended state")
        except:
            print("Bad Read..")
            #accelerateFile.write("Bad Read...\n")
            
        #accelerateFile.write("\n")
            
        #accelerateFile.close()
        #glider.setAngle(30.54584453293809)
    
    
    
# Calculate difference between headings to determine which aileron to raise and cause a roll.
# Check glider current state of roll.
# Determine best angle for turning, and raise or lower aileron to reach desired angle.
# If accelerometer states that the glider is in desired angle for turning,
#  don't raise aileron again.
# Repeat this check as often as possible and keep until headings match.
# When nearing the intended heading, raise opposite aileron to start stabilizing glider.
#
if __name__ == "__main__":
    main()