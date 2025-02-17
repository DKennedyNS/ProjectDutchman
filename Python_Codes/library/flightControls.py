# If you are missing imports, run 'pip install simple_pid' and 'pip install RPi.GPIO'
from simple_pid import PID
import RPi.GPIO as GPIO
from time import sleep

class FlightControls:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    aileronOneAngle = -1.0;
    aileronTwoAngle = -1.0;

    leftOutPin = 27 #left
    rightOutPin = 17 #right
    
    GPIO.setup(leftOutPin, GPIO.OUT)
    GPIO.setup(rightOutPin, GPIO.OUT)
    
    leftPwm=GPIO.PWM(leftOutPin, 50)
    rightPwm = GPIO.PWM(rightOutPin, 50)
    
    pidController = 0
    
    
    def __init__(self):
        self.leftPwm.start(0)
        self.rightPwm.start(0)

        self.setAileronOne(0)
        self.setAileronTwo(0)
        

    # Convert servo angle into a PWM duty cycle so the servo can understand it. Credit: lanc1999 on Instructables.com
    def setAileronOne(self, angle):
        duty1 = (angle+40) / 18 + 2
                
        if self.aileronOneAngle != angle:
        
            GPIO.output(self.leftOutPin, True)
        
            # Break and test
        
            #print("Current Duty: ", duty1)
            self.leftPwm.ChangeDutyCycle(duty1)
        
            #sleep(1)
            GPIO.output(self.leftOutPin, False)
        
            self.aileronOneAngle = angle
        
    def setAileronTwo(self, angle):
        duty2 = (angle+60) / 18 + 2
        
        if self.aileronTwoAngle != angle:
        
            GPIO.output(self.rightOutPin, True)
            # Break and test
        
            #print("Current Duty: ", duty2)
        
            self.rightPwm.ChangeDutyCycle(duty2)
        
            #sleep(1)
        
            GPIO.output(self.rightOutPin, False)
        
            self.aileronTwoAngle = angle
        
    
