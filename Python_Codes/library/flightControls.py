# If you are missing imports, run 'pip install simple_pid' and 'pip install RPi.GPIO'
from simple_pid import PID
import RPi.GPIO as GPIO
from time import sleep

class FlightControls:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    aileronAngle = -1
    oldAileronAngle = -1
    outPin = 2
    GPIO.setup(2, GPIO.OUT)
    pwm=GPIO.PWM(outPin, 50)
    pidController = 0
    def __init__(self):
        self.pwm.start(0)

        # Global vars for aileron angles
        self.aileronAngle = 0
        self.oldAileronAngle = 0
                

    # Convert servo angle into a PWM duty cycle so the servo can understand it. Credit: lanc1999 on Instructables.com
    def setAngle(self, angle):
        #duty = angle / 18 + 2
        duty = (angle+90) / 18 + 2
        GPIO.output(self.outPin, True)
        # Break and test
        #duty = 100
        print("Current doodie: ", duty)
        self.pwm.ChangeDutyCycle(duty)
        sleep(1)
        GPIO.output(self.outPin, False)
        self.pwm.ChangeDutyCycle(0)

    def buildPID(self,currentHDG, targetHDG):
        # Gain values for P, I and D
        Kp = 1
        Ki = 1
        Kd = 1
        
        # Polling rate of the PID controller. Determines how often new commands are sent to servo
        pollRate = 0.1

        # Init the PID Controller with the target heading as the SetPoint. Output bound to +/- 45degrees
        self.pidController = PID(Kp, Ki, Kd, targetHDG, pollRate, [-45, 45], True, False)

        # Get updated output from the PID controller. If it's different, send the new value to the servos
        self.oldAileronAngle = self.aileronAngle
        self.aileronAngle = self.pidController(currentHDG)
        print("Aileron Current Angle: ", self.aileronAngle)
        if self.oldAileronAngle != self.aileronAngle:
            self.setAngle(self.aileronAngle)
        else:
            self.setAngle(0)
            
    def updatePID(self, currentHDG):
        # Get updated output from the PID controller. If it's different, send the new value to the servos
        self.oldAileronAngle = self.aileronAngle
        self.aileronAngle = self.pidController(currentHDG)
        print("inside update: ", currentHDG)
        if self.oldAileronAngle != self.aileronAngle:
            self.setAngle(self.aileronAngle)
            

