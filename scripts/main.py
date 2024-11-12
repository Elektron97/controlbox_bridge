#!/usr/bin/env python
import rospy
from controlbox_bridge.pressure_interface import Pressure_Interface

def main():
	rospy.init_node("controlbox_bridge", anonymous=True)
	Pressure_Interface()
	rospy.spin()

if __name__ == '__main__':
	main()
