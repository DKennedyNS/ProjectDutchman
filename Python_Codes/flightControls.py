# If you are missing imports, run 'pip install simple_pid' and 'pip install RPi.GPIO'
from simple_pid import PID
import RPi.GPIO as GPIO

# Global vars for aileron angles
aileronAngle = 0
oldAileronAngle = 0

# Convert servo angle into a PWM duty cycle so the servo can understand it. Credit: lanc1999 on Instructables.com
def setAngle(angle):

	outPin = 3
	duty = angle / 18 + 2
	GPIO.output(outPin, True)
	pwm.ChangeDutyCycle(duty)
	sleep(1)
	GPIO.output(outPin, False)
	pwm.ChangeDutyCycle(0)


def buildPID(currentHDG, targetHDG):

	# Gain values for P, I and D
	Kp = 1
	Ki = 1
	Kd = 1

	# Polling rate of the PID controller. Determines how often new commands are sent to servo
	pollRate = 0.1

	# Init the PID Controller with the target heading as the SetPoint. Output bound to +/- 45degrees
	pidController = PID(Kp, Ki, Kd, targetHDG, pollRate, [-45, 45], True, False)

	# Get updated output from the PID controller. If it's different, send the new value to the servos
	oldAileronAngle = aileronAngle
	aileronAngle = pidController(currentHDG)
	if oldAileronAngle != aileronAngle:
		setAngle(aileronAngle)

		
def updatePID(currentHDG):
	
	# Get updated output from the PID controller. If it's different, send the new value to the servos
	oldAileronAngle = aileronAngle
	aileronAngle = pidController(currentHDG)
	if oldAileronAngle != aileronAngle:
		setAngle(aileronAngle)
