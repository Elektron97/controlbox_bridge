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
        self.pressures = [0.0] * self.n_chambers
        
        # Put to 0 every chamber
        self.write_pressure(self.pressures)

        # Define Pub/Sub objects
        self.sub_obj = rospy.Subscriber(topic_name, Float32MultiArray, self.pressure_callback)

    def set_communication(self):
        try:
            arduino = serial.Serial(
                port=PORT,
                baudrate=BAUDRATE,
                timeout=TIMEOUT
            )
            arduino.isOpen() 
            print("port is opened!")
        except IOError: 
            arduino.close()
            arduino.open()
            print("port was already open, was closed and opened again!")
        return arduino  
 
    def write_pressure(self, pressures):
        # Saturation & Convert in Digit
        digit_pressures = self.bar2digit(self.saturation(pressures))
  
        # Add syncbyte & create packet
        packet = np.array([SYNCBYTE] + digit_pressures, dtype=np.uint8)

        if self.arduino.isOpen():
            for value in packet: # Sending Data
                s = struct.pack('!{0}B'.format(len(packet)), *packet)
                self.arduino.write(s)

    def saturation(self, pressures, pmax=PMAX, pmin=PMIN):      
        # Safe Saturation
        for i in range(len(pressures)):
            # Saturation on max value
            if pressures[i] > pmax[i]:
                rospy.logwarn_throttle(1.0, "Commanded Pressures higher than the Max Pressure. Saturating...")
                pressures[i] = pmax[i]

            # Deadzone
            elif pressures[i] < pmin[i]:
                rospy.logwarn_throttle(1.0, "Commanded Pressures lower than the Min Pressure. Saturating...")
                pressures[i] = pmin[i]
        return pressures

    def pressure_callback(self, msg):
        # Log
        rospy.loginfo_throttle(1.0, "Writing in the Arduino the commanded pressures...")
  
        # Extract Data
        try:
            if not self.n_chambers == len(msg.data):
                raise ChamberException
            else:
                self.pressures = list(msg.data)
        except ChamberException:
            rospy.logerr("The length of the message ({}) is not consistent with the declared number of chambers ({}).".format(len(msg.data), self.n_chambers))

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