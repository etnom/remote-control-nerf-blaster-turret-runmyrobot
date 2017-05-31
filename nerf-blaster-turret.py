# Made for RunMyRobot
# By Monty C

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_StepperMotor
import time
import atexit
import RPi as GPIO


print "running!"


# Class for Ammo counting and managing
# Traditionally, my ammo counters include 3 components: An ammo counting detection mechanism (IR Gate, or a switch which is oriented to be pressed alongside a trigger), a button to toggle between the various magazine sizes, and a switch to detect when magazines are changed. 
# In this build, I omitted the button to toggle between the various magazine sizes. I would assume there is only going to be one magazine size being used, which can be changed in the code. The magazine changing detection switch is still in the blaster.
class AmmoCounter ():
	def __init__ (self):
		# IO pins
		self.MAGAZINE_INSERTION_DETECTION_PIN = 23
		self.TRIGGER_SWTICh_PIN = 24
		
		# Ammo
		self.currentAmmo = 25
		self.maxAmmo = 25
		
		self.initInputButtons()


	def initInputButtons ():
		# Init magazine insertion detection switch input pin
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.MAGAZINE_INSERTION_DETECTION_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
		GPIO.add_event_detect(self.MAGAZINE_INSERTION_DETECTION_PIN, GPIO.BOTH)
		GPIO.add_event_callback(self.MAGAZINE_INSERTION_DETECTION_PIN, self.reload)

        # Init trigger switch input pin
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.TRIGGER_SWTICh_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
		GPIO.add_event_detect(self.TRIGGER_SWTICh_PIN, GPIO.BOTH)
		GPIO.add_event_callback(self.TRIGGER_SWTICh_PIN, self.countAmmo)

	def countAmmo ():
		if (self.currentAmmo > 0):
			self.currentAmmo = self.currentAmmo - 1

	def reload ():
		self.currentAmmo = self.maxAmmo;


# Class for managing turret
class Turret (): 
	def __init__ (self):
		self.FIRE_PIN = 22
		self.STEPS = 5

		self.ammoCounter = AmmoCounter()

		self.initSteppers().initBlaster()


	# Init stepper motors
	def initSteppers (self):
	 	# new Motor HAT
		self.mh = Adafruit_MotorHAT(addr = 0x60)
		atexit.register(self.disableMotors)

		#create and set stepper motor objects
		self.verticalStepper = self.mh.getStepper(200, 2)
		self.verticalStepper.setSpeed(5)

		self.horizontalStepper = self.mh.getStepper(200, 2)
		self.horizontalStepper.setSpeed(5)

		return self

	# Init GPIO stuff for blaster
	def initBlaster (self):
	    GPIO.setmode(GPIO.BCM)
	    GPIO.setup(FIRE_PIN, GPIO.OUT)
	    GPIO.output(FIRE_PIN, GPIO,LOW)

	    return self

	# Wrapper for step(), so stepping will be easier to manage when multitasking motors
	def stepperWrapper (self, stepper, numOfSteps, direction):
	    stepper.step(numOfSteps, direction, Adafruit_MotorHAT.INTERLEAVE)

	# Functions for aiming/angling/rotating blaster
	# Using threading to be able to control more than 1 motor at the same time
	def rotateUp (self):
		print "rotating up!"

		rotateUp_Thread = threading.Thread(target = self.stepperWrapper, args = (self.verticalStepper, self.STEPS, Adafruit_MotorHAT.FORWARD))
		rotateUp_Thread.start()
		return self

	def rotateDown (self):
		print "rotating down!"

		rotateDown_Thread = threading.Thread(target = self.stepperWrapper, args = (self.verticalStepper, self.STEPS, Adafruit_MotorHAT.BACKWARD))
		rotateDown_Thread.start()
		return self

	def rotateRight ():
		print "rotating right!"

		rotateRight_Thread = threading.Thread(target = self.stepperWrapper, args = (self.horizontalStepper, self.STEPS, Adafruit_MotorHAT.FORWARD))
		rotateRight_Thread.start()
		return self

	def rotateLeft ():
		print "rotating left!"

		rotateLeft_Thread = threading.Thread(target = self.stepperWrapper, args = (self.horizontalStepper, self.STEPS, Adafruit_MotorHAT.BACKWARD))
		rotateLeft_Thread.start()
		return self

	# auto-disable motors on shutdown
	def disableMotors():
		self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

	def shoot():
		print "shooting!"

		self.lastAmmo = ammoCounter.currentAmmo
		if (ammoCounter.currentAmmo >= self.lastAmmo):
			GPIO.output(RELAY_PIN, GPIO.HIGH)
		else: 
			GPIO.output(RELAY_PIN, GPIO.LOW)
		return self

turret = Turret();

# main loop
while (True):
    turret.rotateUp()
    turret.rotateUp()
    turret.rotateUp()
    turret.rotateDown()
    turret.rotateDown()
    turret.rotateDown()





