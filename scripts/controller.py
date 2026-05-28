#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

# ==========================================
# CONFIGURATION & CONSTANTS
# ==========================================
N_MOTORS = 6

GAIN = 3.0
MOTOR_SF = -(2.0 / 3.0) * math.sqrt(3.0)
# MOTOR_SF *= GAIN

# Phases
PHASE0 = 0.0
PHASE1 = 2.0 * math.pi / 3.0
PHASE2 = 4.0 * math.pi / 3.0


PHASE_DISP = math.pi / 6.0

class JoystickAxes:
    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    RIGHT_STICK_X = 3
    RIGHT_STICK_Y = 4

# ==========================================
# PRECOMPUTED MATH CONSTANTS
# ==========================================
# By precomputing these denominators combined with the scale factor,
# we eliminate 3 sine operations, 3 subtractions, and 3 divisions per function call.

def precompute_phase_denominators(p1, p2, p3):
    return (
        math.sin(p1 - p2) * -MOTOR_SF,
        math.sin(p2 - p3) * -MOTOR_SF,
        math.sin(p1 - p3) * -MOTOR_SF
    )

# Constants for First Module
MOD1_P1, MOD1_P2, MOD1_P3 = PHASE0, PHASE1, PHASE2
MOD1_D12, MOD1_D23, MOD1_D13 = precompute_phase_denominators(MOD1_P1, MOD1_P2, MOD1_P3)

# Constants for Second Module
MOD2_P1, MOD2_P2, MOD2_P3 = PHASE0 + PHASE_DISP, PHASE1 + PHASE_DISP, PHASE2 + PHASE_DISP
MOD2_D12, MOD2_D23, MOD2_D13 = precompute_phase_denominators(MOD2_P1, MOD2_P2, MOD2_P3)

# ==========================================
# MATH UTILITY LIBRARY
# ==========================================

# def cartesian2Polar(x, y):
#     """Converts Cartesian coordinates to Polar using optimized math.hypot."""
#     return math.hypot(x, y), math.atan2(y, x) + math.pi

def cartesian2Polar(x, y):
    """
    Converts Cartesian coordinates to Polar, applying Elliptical Grid Mapping 
    to map a square joystick boundary into a perfect circle (max rho = 1.0).
    """
    # 1. Map the square coordinates to circular coordinates
    x_circle = x * math.sqrt(1.0 - (y**2) / 2.0)
    y_circle = y * math.sqrt(1.0 - (x**2) / 2.0)
    
    # 2. Calculate Polar coordinates using the mapped values
    rho = math.hypot(x_circle, y_circle)
    theta = math.atan2(y_circle, x_circle) + math.pi
    
    return rho, theta

def map2motors_optimized(rho, theta, p1, p2, p3, d12, d23, d13):
    """Maps polar coordinates to motor commands using precomputed denominators."""
    rho1 = rho2 = rho3 = 0.0
    
    # Sector 1: phi \in [p1, p2]
    if p1 <= theta < p2:
        rho1 =  rho * math.sin(theta - p2) / d12
        rho2 = -rho * math.sin(theta - p1) / d12
    
    # Sector 2: phi \in [p2, p3]
    elif p2 <= theta < p3:
        rho2 =  rho * math.sin(theta - p3) / d23
        rho3 = -rho * math.sin(theta - p2) / d23
    
    # Sector 3: phi \in [p3, 2*pi] or Sector 0: phi \in [0, p1]
    elif p3 <= theta <= 2.0 * math.pi or 0.0 <= theta < p1:
        rho1 =  rho * math.sin(theta - p3) / d13
        rho3 = -rho * math.sin(theta - p1) / d13
    else:
        rospy.logerr_throttle(1.0, "Theta range not valid. Phase must be in [0, 2*pi]")

    return rho1, rho2, rho3

# ==========================================
# ROS CONTROL NODE CLASS
# ==========================================

class ControlNode:
    def __init__(self):
        rospy.init_node('controller', anonymous=False)
        
        # Init vector objs (Pre-allocate to avoid garbage collection overhead in loops)
        self.turn_commands = Float32MultiArray()
        self.turn_commands.data = [0.0] * N_MOTORS

        # Setup Publishers
        self.pub_turns = rospy.Publisher('/pressures', Float32MultiArray, queue_size=1)

        # Setup Subscriber (queue_size=1 drops old joystick inputs if the loop falls behind)
        self.sub_joy = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)

        # Setup Main Loop Timer (10Hz / 0.1s)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.main_loop)
        
        rospy.loginfo("Control Node initialized.")

    def joy_callback(self, msg):
        # --- Extract Information from LEFT stick ---
        rho_left, theta_left = cartesian2Polar(
            msg.axes[JoystickAxes.LEFT_STICK_X], 
            -msg.axes[JoystickAxes.LEFT_STICK_Y]
        )

        # --- Extract Information from RIGHT stick ---   
        rho_right, theta_right = cartesian2Polar(
            msg.axes[JoystickAxes.RIGHT_STICK_X], 
            -msg.axes[JoystickAxes.RIGHT_STICK_Y]
        )

        # --- Update motor_cmd array in place ---
        # First Module (Indices 0, 1, 2)
        r1, r2, r3 = map2motors_optimized(
            GAIN*rho_left, theta_left, 
            MOD1_P1, MOD1_P2, MOD1_P3, 
            MOD1_D12, MOD1_D23, MOD1_D13
        )
        
        # Second Module (Indices 3, 4, 5)
        r4, r5, r6 = map2motors_optimized(
            GAIN*rho_right, theta_right, 
            MOD2_P1, MOD2_P2, MOD2_P3, 
            MOD2_D12, MOD2_D23, MOD2_D13
        )

        # Reassign data tuple (ROS requires replacing the sequence, but a tuple is highly efficient)
        self.turn_commands.data = (r1, r2, r3, r4, r5, r6)

    def main_loop(self, event):
        # Publish /cmd_turns
        self.pub_turns.publish(self.turn_commands)

    def on_shutdown(self):
        rospy.loginfo("Control Node turning off.")

if __name__ == '__main__':
    try:
        node = ControlNode()
        rospy.on_shutdown(node.on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End of Main")
