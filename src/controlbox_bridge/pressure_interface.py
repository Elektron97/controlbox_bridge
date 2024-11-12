### Pressure Interface Class ###

import rospy
from std_msgs.msg import Float32MultiArray

import numpy as np
import serial
import struct

# Parameter for hardware setup 
N_CHAMBERS = rospy.get_param('hardware_params/n_chambers')
PMAX      = rospy.get_param('hardware_params/pmax')
PMIN      = rospy.get_param('hardware_params/pmin')
MAX_DIGIT = rospy.get_param('hardware_params/digit_max')
MIN_DIGIT = rospy.get_param('hardware_params/digit_min')

# Serial Communication
BAUDRATE = rospy.get_param('serial_params/baudrate')
TIMEOUT  = rospy.get_param('serial_params/timeout')
PORT     = rospy.get_param('serial_params/port')
SYNCBYTE = rospy.get_param('serial_params/syncbyte')

# Topic Names
topic_name = '/pressures'

class ChamberException(Exception):
	pass

class Pressure_Interface(object): 
	def __init__(self):
		# Arduino Obj
		self.arduino = self.set_communication()
		
		# Parameters of the class
		self.n_chambers = N_CHAMBERS

		# Define Pressure Array
		self.pressures = [0.0]*self.n_chambers
		
  		# Put to 0 every chambers
		self.write_pressure(self.pressures)

		# Define Pub/Sub objects
		self.sub_obj = rospy.Subscriber(topic_name, Float32MultiArray, self.pressure_callback)

	def set_communication(self):
		try:
			arduino = serial.Serial( # set parameters, in fact use your own :-)
				port=PORT,
				baudrate=BAUDRATE,
				timeout=TIMEOUT
			)
			arduino.isOpen() # try to open port, if possible print message
			print ("port is opened!")
		except IOError: # if port is already opened, close it and open it again and print message
			arduino.close()
			arduino.open()
			print ("port was already open, was closed and opened again!")
		return arduino	
 
	def write_pressure(self, pressures):
		#########################################################
		# We pass "pressures" to avoid changes					#
		# during the execution of the methods,					#
		# due to the callback or other types of interruptions.	#
		#########################################################
  
		# Saturation & Convert in Digit
		digit_pressures = self.bar2digit(self.saturation(pressures))
  
		# Add syncbyte & create packet
		packet = np.array([SYNCBYTE] + digit_pressures, dtype = np.uint8)

		if self.arduino.isOpen():
			for value in packet: # Sending Data
				s = struct.pack('!{0}B'.format(len(packet)), *packet)
				self.arduino.write(s)

	def saturation(self, pressures, pmax = PMAX, pmin = PMIN):		
		# Safe Saturation
		for i in range(len(pressures)):
			# Saturation on max value
			if pressures[i] > pmax[i]:
				rospy.logwarn("Commanded Pressures higher than the Max Pressure. Saturating...")
				pressures[i] = pmax[i]

			# Deadzone
			elif pressures[i] < pmin[i]:
				rospy.logwarn("Commanded Pressures lower than the Min Pressure. Saturating...")
				pressures[i] = pmin[i]
				
			else:
				pass
		return pressures

	def pressure_callback(self, msg):
		# Log
		rospy.loginfo("Writing in the Arduino the commanded pressures...")
  
  		# Extract Data
		try:
			if not self.n_chambers == len(msg.data):
				raise ChamberException
			else:
				self.pressures = list(msg.data)
		except ChamberException:
			rospy.logerr("The length of the message ({}) is not consinstent with the declared number of chambers ({}).".format(len(msg.data), self.n_chambers))

		# Send to Arduino
		self.write_pressure(self.pressures)
 
	def __del__(self):
		# Set to 0 Pressure Array
		self.pressures = [0.0]*self.n_chambers
		# Put to 0 every chambers
		self.write_pressure(self.pressures)
  
		# Close Arduino Communication
		if self.arduino:
			self.arduino.close()
    
		# Unregister subscription
		if self.sub_obj:
			self.sub_obj.unregister()

		print("Object destroyed succesfully! ")
		
	def bar2digit(self, bar):
     
		#####################################################################
		# 																	#
		#			Function to convert the pressure from bar to digit:		#
		#																	#
		#	p_digit = (max_digit - min_digit) * (p / max_bar) + min_digit	#
		#																	#
		#				p_digit 	= pressure value in digit				#
		#				p 			= pressure value in bar					#
		# 				max_digit 	= max value of pressure in digit		#
		# 				min_digit	= min value of pressure in digit		#
		#				max_bar		= max value of pressure in bar			#
		#				int() = function to convert from double to int  	#
		#																	#
		#####################################################################
		digit = []		
  
		for i in range(self.n_chambers):
			digit.append(int((MAX_DIGIT[i] - MIN_DIGIT[i]) * (bar[i] / PMAX[i]) + MIN_DIGIT[i]))
        
		return digit

