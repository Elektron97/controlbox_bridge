#!/usr/bin/env python
import rospy
from controlbox_bridge.pressure_interface import Pressure_Interface

def main():
    rospy.init_node("controlbox_bridge", anonymous=False)
    
    # 1. Initialize your hardware interface
    pi = Pressure_Interface()
    
    # 2. Tell ROS to trigger the shutdown method inside your class when Ctrl+C is pressed
    rospy.on_shutdown(pi.shutdown_hook)
    
    # 3. Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        # This catches the Ctrl+C interruption gracefully
        rospy.loginfo("Node interrupted. Exiting safely...")