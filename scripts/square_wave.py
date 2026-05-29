#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse
import threading

# ==========================================
# CONFIGURATION & CONSTANTS
# ==========================================
N_MOTORS = 6

# Default parameters for the square wave
DEFAULT_RATE = 100.0
DEFAULT_PERIOD = 1.0  # Time in seconds for one full wave cycle
DEFAULT_RHO = 3.0
DEFAULT_THETA = math.pi / 2.0

# Motor Math Constants
MOTOR_SF = -(2.0 / 3.0) * math.sqrt(3.0)

# Phases
PHASE0 = 0.0
PHASE1 = 2.0 * math.pi / 3.0
PHASE2 = 4.0 * math.pi / 3.0
PHASE_DISP = 0.0

# ==========================================
# PRECOMPUTED MATH CONSTANTS
# ==========================================
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
def map2motors_optimized(rho, theta, p1, p2, p3, d12, d23, d13):
    """Maps polar coordinates to motor commands using precomputed denominators."""
    rho1 = rho2 = rho3 = 0.0
    
    # Ensure theta is purely bounded between 0 and 2*pi
    theta = theta % (2.0 * math.pi)
    
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
# ROS PUBLISHER CLASS
# ==========================================
class PolarSquareWavePublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/pressures', Float32MultiArray, queue_size=10)
        
        # ROS Parameters
        self.rate_hz = rospy.get_param('~rate', DEFAULT_RATE)
        self.period = rospy.get_param('~period', DEFAULT_PERIOD)
        self.rho = rospy.get_param('~rho', DEFAULT_RHO)
        self.theta = rospy.get_param('~theta', DEFAULT_THETA)

        self.running = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.daemon = True
        self.thread.start()

        # Services
        self.start_srv = rospy.Service('/start', Trigger, self.start_callback)
        self.stop_srv = rospy.Service('/stop', Trigger, self.stop_callback)

    def start_callback(self, req):
        rospy.loginfo("Polar square wave start requested. Resetting time after 1 second...")
        with self.lock:
            self.start_time = None
            self.running = True
        rospy.sleep(1.0)
        with self.lock:
            self.start_time = rospy.Time.now()
        return TriggerResponse(success=True, message="Polar square wave started.")

    def stop_callback(self, req):
        rospy.loginfo("Polar square wave stop requested.")

        with self.lock:
            self.running = False

        # Publish one last message of zeros for safety
        zero_msg = Float32MultiArray()
        zero_msg.data = [0.0] * N_MOTORS
        self.pub.publish(zero_msg)
        rospy.loginfo("Published zero message after stopping wave.")
        rospy.sleep(0.1)

        return TriggerResponse(success=True, message="Wave stopped and zero message published.")

    def publish_loop(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            with self.lock:
                if self.running and self.start_time is not None:
                    t = (rospy.Time.now() - self.start_time).to_sec()

                    # Determine square wave state
                    # First half of period = State 0 (theta), Second half = State 1 (theta + pi)
                    if (t % self.period) < (self.period / 2.0):
                        current_theta = self.theta
                    else:
                        current_theta = self.theta + math.pi

                    # Wrap theta strictly to [0, 2*pi] inside the mapping function
                    # Calculate for Module 1
                    r1, r2, r3 = map2motors_optimized(
                        self.rho, current_theta, 
                        MOD1_P1, MOD1_P2, MOD1_P3, 
                        MOD1_D12, MOD1_D23, MOD1_D13
                    )
                    
                    # Calculate for Module 2
                    r4, r5, r6 = map2motors_optimized(
                        self.rho, current_theta, 
                        MOD2_P1, MOD2_P2, MOD2_P3, 
                        MOD2_D12, MOD2_D23, MOD2_D13
                    )

                    msg = Float32MultiArray()
                    msg.data = [r1, r2, r3, r4, r5, r6]
                    self.pub.publish(msg)
                    
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('polar_square_wave_node')
    PolarSquareWavePublisher()
    rospy.loginfo("Polar square wave publisher node started.")
    rospy.spin()