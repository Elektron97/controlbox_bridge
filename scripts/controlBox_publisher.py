# !usr/bin/env python3
## it indicates that this is a python source file

import rospy
import serial
import struct
import time
import numpy as np
from std_msgs.msg import Float32
import serial.tools.list_ports as ports

class controlBox_pressure:
    def __init__(self):
        # ------------------------ CONSTANTS ------------------------ #
        self.PMAX = 13 # which maps to about 1.4 bars
        self.arduinoPB = None

    def set_communication(self, aportPB, baudrate):
        try:
            self.arduinoPB = serial.Serial( 
                                port=aportPB,
                                baudrate=baudrate,
                                timeout=0.1 
                                )
            self.arduinoPB.isOpen() # try to open port
            # print ("Valve is opened!")
        except IOError: # if port is already opened, close it and open it again and print message
            self.arduinoPB.close()
            self.arduinoPB.open()
            print ("Valve was already open, was closed and opened again!")

    ## Send values to the Pressure Box
    def send_values(self, U1, U2, U3, L1, L2, L3, G, PMAX):
        if U1 > PMAX or U2 > PMAX or U3 > PMAX or L1 > PMAX or L2 > PMAX or L3 > PMAX or G > 1:
            print("Pressure out of range")
            return

        packet = np.array([106,U1,U2,U3,L1,L2,L3,G],dtype = np.uint8)
        
        if self.arduinoPB.isOpen():
            for value in packet : 
                s = struct.pack('!{0}B'.format(len(packet)), *packet)
                self.arduinoPB.write(s)
    
    def valve_shutdown(self):
        print("Shutting down the valve!!")
        self.send_values(0,0,0,0,0,0,0,PMAX) # Release the object
        self.arduinoPB.close()        

    def actuate_finger(self, pressure_value, aportPB, baudrate, PMAX):
        try:
            start_time = time.time()
            self.set_communication(aportPB, baudrate) # open the port
            print("Grasp!!")
            self.send_values(pressure_value,0,0,0,0,0,0,PMAX)  # Open the valve V1 and grasp the object
            time.sleep(3)
            self.send_values(0,0,0,0,0,0,0,PMAX) # Release the object
            time.sleep(2)
            print("Release!!")
            end_time = time.time()
            total_time = (end_time - start_time)
            print(f"Total time taken for the experiment is {total_time}")
            return pressure_value[0]

        except Exception as e:
            print(f"Error encountered while sending value to control Box, closing the port!!")
            self.send_values(0,0,0,0,0,0,0,PMAX) # Release the object
            self.arduinoPB.close()

publisherNodeName = "controlBox_publisher"
topicName = "controlBox_topic"
frequency = 0.5 # 20 hz
baudrate = 115200
pressure_value = 12
PMAX = 13
port = '/dev/ttyACM0'
controlBox_obj = controlBox_pressure()

## initialize the node
rospy.init_node(publisherNodeName, anonymous=True)
publisher = rospy.Publisher(topicName, Float32, queue_size=60)

## specify the rate/frequency of transmission
rate = rospy.Rate(frequency)

while not rospy.is_shutdown():
    # send value to the control box
    pressure_values = controlBox_obj.actuate_finger(pressure_value, port, baudrate, PMAX)
    rospy.loginfo(f"Publishing pressure values!!")
    publisher.publish(pressure_values)
    # we wait for certain amount of time to make sure that the specified transmission rate is achieved
    rate.sleep()
 
    if rospy.is_shutdown():
        print("Encountered!!")
        # close the valve again
        controlBox_obj.valve_shutdown()

