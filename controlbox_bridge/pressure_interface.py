### Pressure Interface Class ###
import rclpy
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from rclpy.node import Node

import numpy as np
import serial
import struct

class PressureInterface(Node):
    #TODO: move everything to numpy arrays for efficiency
    #TODO: use two file for the parameters, one for the hardware setup and one for the serial communication
    def __init__(self):
        super().__init__('pressure_interface')

        # Declare parameters
        ## hardware setup 
        self.declare_parameter('n_chambers', 8)
        self.declare_parameter('p_max', [1.0]*8)
        self.declare_parameter('p_min', [0.0]*8)
        self.declare_parameter('digit_max', [255]*8)
        self.declare_parameter('digit_min', [0]*8)

        ## serial communication
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('syncbyte', 255)

        # Get parameters
        self.n_chambers = self.get_parameter('n_chambers').get_parameter_value().integer_value
        self.pmax = self.get_parameter('p_max').get_parameter_value().double_array_value
        self.pmin = self.get_parameter('p_min').get_parameter_value().double_array_value
        self.digit_max = self.get_parameter('digit_max').get_parameter_value().integer_array_value
        self.digit_min = self.get_parameter('digit_min').get_parameter_value().integer_array_value

        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.syncbyte = self.get_parameter('syncbyte').get_parameter_value().integer_value

        # Topic Names
        self.topic_name = '/pressures'

        self.get_logger().info(f"Parameters: n_chambers={self.n_chambers}, pmax={self.pmax}, pmin={self.pmin}, digit_max={self.digit_max}, digit_min={self.digit_min}, baudrate={self.baudrate}, timeout={self.timeout}, port={self.port}, syncbyte={self.syncbyte}")
        
        # Arduino initialization
        self.arduino = self.set_communication()

        # Define Pressure Array
        self.pressures = [0.0]*self.n_chambers

        # Put to 0 every chambers
        self.write_pressure(self.pressures)

        # Subscriber to set pressures
        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.topic_name,
            self.pressure_callback,
            10
        )

        # Service for resetting the control box
        self.reset_service = self.create_service(
            Empty,
            'reset_control_box',
            self.reset_control_box_callback
        )

    def reset_control_box_callback(self, request, response):
        try:
            self.reset()
            self.get_logger().info('Control box reset successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to reset control box: {e}')
        return response

    def reset(self):
        # Set to 0 Pressure Array
        self.pressures = [0.0]*self.n_chambers
        # Put to 0 every chambers
        self.write_pressure(self.pressures)
        self.get_logger().info("Control box reset to 0 pressure.")
    
    def set_communication(self):
        try:
            arduino = serial.Serial( # set parameters, in fact use your own :-)
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            arduino.isOpen() # try to open port, if possible print message
            self.get_logger().info("port is opened!")

        except IOError: # if port is already opened, close it and open it again and print message
            arduino.close()
            arduino.open()
            self.get_logger().info("port was already open, was closed and opened again!")
        return arduino

    def write_pressure(self, pressures):
        #########################################################
        # We pass "pressures" to avoid changes                 #
        # during the execution of the methods,                #
        # due to the callback or other types of interruptions.#
        #########################################################
  
        # Saturation & Convert in Digit
        digit_pressures = self.bar2digit(self.saturation(pressures))
  
        # Add syncbyte & create packet
        packet = np.array([self.syncbyte] + digit_pressures, dtype = np.uint8)

        if self.arduino.isOpen():
            for value in packet: # Sending Data
                s = struct.pack('!{0}B'.format(len(packet)), *packet)
                self.arduino.write(s)

    def saturation(self, pressures):
        # Safe Saturation
        for i in range(len(pressures)):
            # Saturation on max value
            if pressures[i] > self.pmax[i]:
                self.get_logger().warn("Commanded Pressures higher than the Max Pressure. Saturating...")
                pressures[i] = self.pmax[i]

            # Deadzone
            elif pressures[i] < self.pmin[i]:
                self.get_logger().warn("Commanded Pressures lower than the Min Pressure. Saturating...")
                pressures[i] = self.pmin[i]
                
            else:
                pass
        return pressures

    def bar2digit(self, bar):
     
        #####################################################################
        #                                                                   #
        #           Function to convert the pressure from bar to digit:    #
        #                                                                   #
        #   p_digit = (max_digit - min_digit) * (p / max_bar) + min_digit  #
        #                                                                   #
        #               p_digit     = pressure value in digit               #
        #               p           = pressure value in bar                 #
        #               max_digit   = max value of pressure in digit        #
        #               min_digit   = min value of pressure in digit        #
        #               max_bar     = max value of pressure in bar          #
        #               int() = function to convert from double to int      #
        #                                                                   #
        #####################################################################
        digit = []        
  
        for i in range(self.n_chambers):
            digit.append(int((self.digit_max[i] - self.digit_min[i]) * (bar[i] / self.pmax[i]) + self.digit_min[i]))
        
        return digit

    def pressure_callback(self, msg):
        try:
            if len(msg.data) != self.n_chambers:
                raise ChamberException(f"Expected {self.n_chambers} pressure values, but got {len(msg.data)}.")
            
            self.pressures = list(msg.data)
            self.write_pressure(self.pressures)
            self.get_logger().info(f"Set pressures to: {self.pressures}")

        except ChamberException as e:
            self.get_logger().error(str(e))
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")

    def destroy_node(self):
        self.get_logger().info("Destroying Pressure Interface Node...")

        # Reset the control box
        self.reset()
  
        # Close Arduino Communication
        if self.arduino:
            self.arduino.close()

        self.get_logger().info("Pressure Interface Node destroyed.")
        super().destroy_node()

class ChamberException(Exception):
    pass
