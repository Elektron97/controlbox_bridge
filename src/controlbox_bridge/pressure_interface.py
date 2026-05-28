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

PSAFE = 3.0

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
		self.sub_obj = rospy.Subscriber(topic_name, Float32MultiArray, self.pressure_callback, queue_size=1)

	def set_communication(self):
		try:
			arduino = serial.Serial( # set parameters, in fact use your own :-)
				port=PORT,
				baudrate=BAUDRATE,
				timeout=TIMEOUT,
				write_timeout=TIMEOUT
			)
			arduino.isOpen() # try to open port, if possible print message
			print ("port is opened!")
		except IOError: # if port is already opened, close it and open it again and print message
			arduino.close()
			arduino.open()
			print ("port was already open, was closed and opened again!")
		return arduino
 
	# def write_pressure(self, pressures):
	# 	press_copy = list(pressures)
		
	# 	# Saturazione & Conversione
	# 	digit_pressures = self.bar2digit(self.saturation(press_copy, pmax=[PSAFE]*self.n_chambers))
		
	# 	if self.arduino.isOpen():
	# 		try:
	# 			# Creiamo il formato in modo dinamico
	# 			pack_format = '!' + 'B' * (1 + self.n_chambers)
	# 			s = struct.pack(pack_format, SYNCBYTE, *digit_pressures)
				
	# 			# IL TRUCCO È QUI: 
	# 			# Svuota (elimina) tutti i vecchi comandi in attesa di essere spediti
	# 			self.arduino.reset_output_buffer()
				
	# 			# Ora scrivi il nuovo comando, che sarà il primo e unico nella coda
	# 			self.arduino.write(s)
				
	# 		except serial.SerialTimeoutException:
	# 			rospy.logwarn_throttle(1.0, "Timeout in scrittura! Pacchetto scartato.")
	# 		except serial.SerialException as e:
	# 			rospy.logerr("Errore seriale: {}".format(e))
 
	def write_pressure(self, pressures):
		press_copy = list(pressures)
		
		# 1. Saturazione & Conversione 
		digit_pressures = self.bar2digit(self.saturation(press_copy, pmax=[PSAFE]*self.n_chambers))
		
		# 2. PADDING (Assicuriamo che ci siano sempre ESATTAMENTE 7 byte di dati)
		# Se abbiamo meno di 7 camere, riempiamo il resto con zeri (0)
		while len(digit_pressures) < 7:
			digit_pressures.append(0)
			
		# Se per sbaglio ne abbiamo più di 7, tronchiamo la lista
		if len(digit_pressures) > 7:
			digit_pressures = digit_pressures[:7]
		
		if self.arduino.isOpen():
			try:
				# Il formato ora è SEMPRE 8 byte (1 Sync + 7 Dati)
				# '!B' (Sync) + 'BBBBBBB' (7 Dati)
				pack_format = '!BBBBBBBB' 
				s = struct.pack(pack_format, SYNCBYTE, *digit_pressures)
				
				# 3. TIRIAMO LO SCIACQUONE (Salviamo Arduino dal blocco)
				# Leggiamo e buttiamo via tutto quello che Arduino ci ha mandato con ardprintf
				if self.arduino.in_waiting > 0:
					self.arduino.reset_input_buffer()
				
				# Svuotiamo anche il buffer di uscita per evitare latenza
				self.arduino.reset_output_buffer()
				
				# Invia il comando ad Arduino
				self.arduino.write(s)
				
			except serial.SerialTimeoutException:
				rospy.logwarn_throttle(1.0, "Timeout scrittura. Il buffer è pieno, salto il pacchetto.")
			except serial.SerialException as e:
				rospy.logerr("Errore seriale: {}".format(e))

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
 
    # --> FIXED INDENTATION HERE <--
	def shutdown_hook(self):
		rospy.loginfo("ROS Shutdown initiated. Safely turning off hardware...")
		try:
            # Set to 0 Pressure Array
			self.pressures = [0.0] * self.n_chambers
            # Put to 0 every chamber
			self.write_pressure(self.pressures)
			rospy.loginfo("Hardware pressures set to safe state (0.0).")
		except Exception as e:
			rospy.logerr("Failed to send zero pressures during shutdown: {}".format(e))

        # Close Arduino Communication safely
		if hasattr(self, 'arduino') and self.arduino:
			if self.arduino.isOpen():
				self.arduino.close()
				rospy.loginfo("Arduino serial communication closed.")
        
        # Unregister subscription
		if hasattr(self, 'sub_obj') and self.sub_obj:
			self.sub_obj.unregister()

		rospy.loginfo("Pressure Interface object destroyed successfully!")
        
	def bar2digit(self, bar):
		digit = []
		for i in range(self.n_chambers):
			digit.append(int((MAX_DIGIT[i] - MIN_DIGIT[i]) * (bar[i] / PMAX[i]) + MIN_DIGIT[i]))
		return digit
