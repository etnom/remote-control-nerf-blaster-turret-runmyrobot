# Made for RunMyRobot
# By Monty C

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_StepperMotor, Adafruit_DCMotor
import time
import atexit
import RPi.GPIO as GPIO
import threading


# Wrapper for step(), so stepping will be easier to manage when multitasking motors
def stepperWrapper (self, stepper, numOfSteps, direction):
    stepper.step(numOfSteps, direction, Adafruit_MotorHAT.INTERLEAVE)

# Class for Ammo counting and managing
# Traditionally, my ammo counters include 3 components: An ammo counting detection mechanism (IR Gate, or a switch which is oriented to be pressed alongside a trigger), a button to toggle between the various magazine sizes, and a switch to detect when magazines are changed. 
# In this build, I omitted the button to toggle between the various magazine sizes. I would assume there is only going to be one magazine size being used, which can be changed in the code. The magazine changing detection switch is still in the blaster.
# class AmmoCounter ():
# 	def __init__ (self):
# 		# IO pins
# 		self.MAGAZINE_INSERTION_DETECTION_PIN = 22
# 		# self.TRIGGER_SWTICh_PIN = 24
		
# 		# Ammo
# 		self.currentAmmo = 25
# 		self.maxAmmo = 25
		
# 		# self.initInputButtons()


# 	def initInputButtons (self):
# 		# Init magazine insertion detection switch input pin
# 		# GPIO.setmode(GPIO.BCM)
# 		# GPIO.setup(self.MAGAZINE_INSERTION_DETECTION_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
# 		# GPIO.add_event_detect(self.MAGAZINE_INSERTION_DETECTION_PIN, GPIO.BOTH)
# 		#GPIO.add_event_callback(self.MAGAZINE_INSERTION_DETECTION_PIN, self.reloadAmmo)

#         # Init trigger switch input pin
# 		# GPIO.setmode(GPIO.BCM)
# 		# GPIO.setup(self.TRIGGER_SWTICh_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
# 		# GPIO.add_event_detect(self.TRIGGER_SWTICh_PIN, GPIO.BOTH)
# 		# GPIO.add_event_callback(self.TRIGGER_SWTICh_PIN, self.countAmmo)

# 	def countAmmo (self):
# 		if (self.currentAmmo > 0):
# 			self.currentAmmo = self.currentAmmo - 1

# 	def reloadAmmo (self):
# 		self.currentAmmo = self.maxAmmo;
# 		print "reloading"


# Class for managing turret
class Turret (): 
	def __init__ (self):
		self.FLYWHEEL_PIN = 24
		self.FIRE_PIN = 23
		self.STEPS = 5

		# self.ammoCounter = AmmoCounter()

		self.initMotors().initBlaster()


	# Init stepper motors
	def initMotors (self):
	 	# new Motor HAT
		self.mh = Adafruit_MotorHAT(addr = 0x60)
		atexit.register(self.disableTurret)

		#create and set stepper motor objects
		self.verticalStepper = self.mh.getStepper(200, 1)
		self.verticalStepper.setSpeed(5)

		self.horizontalMotor = self.mh.getMotor(3)
		self.horizontalMotor.setSpeed(150)

		return self

	# Init GPIO stuff for blaster
	def initBlaster (self):
		#pin for flywheels
		#always have flywheels on. It will be noisy, but there will be no delay when firing since we dont need to keep toggling the flywheels
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.FLYWHEEL_PIN, GPIO.OUT)
		GPIO.output(self.FLYWHEEL_PIN, GPIO.LOW)
	    
		#pin for firing
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.FIRE_PIN, GPIO.OUT)
		GPIO.output(self.FIRE_PIN, GPIO.HIGH)
	
		return self

	# Functions for aiming/angling/rotating blaster
	# Using threading to be able to control more than 1 motor at the same time
	def rotateDown (self):
		print "rotating up!"

		# rotateUp_Thread = threading.Thread(target = stepperWrapper, args = (self.verticalStepper, self.STEPS, Adafruit_MotorHAT.FORWARD))
		# rotateUp_Thread.start()
		
		self.verticalStepper.step(self.STEPS, Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.INTERLEAVE)
		return self

	def rotateUp (self):
		print "rotating down!"

		# rotateDown_Thread = threading.Thread(target = stepperWrapper, args = (self.verticalStepper, self.STEPS, Adafruit_MotorHAT.BACKWARD))
		# rotateDown_Thread.start
		
		self.verticalStepper.step(self.STEPS, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.INTERLEAVE)
		return self

	def rotateRight (self):
		print "rotating right!"

		# rotateRight_Thread = threading.Thread(target = stepperWrapper, args = (self.horizontalStepper, self.STEPS, Adafruit_MotorHAT.FORWARD))
		# rotateRight_Thread.start()
		
		self.horizontalMotor.run(Adafruit_MotorHAT.BACKWARD)
		return self

	def rotateLeft (self):
		print "rotating left!"

		# rotateLeft_Thread = threading.Thread(target = stepperWrapper, args = (self.horizontalStepper, self.STEPS, Adafruit_MotorHAT.BACKWARD))
		# rotateLeft_Thread.start()
		
		self.horizontalMotor.run(Adafruit_MotorHAT.FORWARD)
		return self

	#auto disable all motors and relays on shutdown
	def disableTurret (self):
		self.disableStepperMotors()

	# auto-disable motors on shutdown
	def disableStepperMotors(self):
		self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
		
		return self
		
	#disable blaster. This includes shutting off relay and clearing GPIO
	def disableBlaster(self):
		GPIO.output(self.FIRE_PIN, GPIO.LOW)
		GPIO.output(self.FLYWHEEL_PIN, GPIO.LOW)
		
		GPIO.cleanup()
		
		return self

	def shoot(self):
		print "shooting! from nerfBlasterTurret"

		GPIO.output(self.FIRE_PIN, GPIO.LOW)
		time.sleep(.3)
		GPIO.output(self.FIRE_PIN, GPIO.HIGH)
		
		return self
