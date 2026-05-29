#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse
import threading
import time

DEFAULT_ALPHA = 0.01
DEFAULT_RATE = 100.0
DEFAULT_AMPLITUDE = 3.0
N_MOTORS = 6

class ChirpPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/pressures', Float32MultiArray, queue_size=10)
        self.alpha = rospy.get_param('~alpha', DEFAULT_ALPHA)
        self.rate_hz = rospy.get_param('~rate', DEFAULT_RATE)
        self.amplitude = rospy.get_param('~amplitude', DEFAULT_AMPLITUDE)
        self.running = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.daemon = True
        self.thread.start()

        # Services
        self.start_srv = rospy.Service('/start', Trigger, self.start_callback)
        self.stop_srv = rospy.Service('/stop', Trigger, self.stop_callback)

    def start_callback(self, req):
        rospy.loginfo("Chirp start requested. Resetting time after 1 second...")
        with self.lock:
            self.start_time = None
            self.running = True
        rospy.sleep(1.0)
        with self.lock:
            self.start_time = rospy.Time.now()
        return TriggerResponse(success=True, message="Chirp started.")

    def stop_callback(self, req):
        rospy.loginfo("Chirp stop requested.")

        with self.lock:
            self.running = False

        # Publish one last message of zeros
        zero_msg = Float32MultiArray()
        zero_msg.data = [0.0] * N_MOTORS
        self.pub.publish(zero_msg)
        rospy.loginfo("Published zero message after stopping chirp.")
        rospy.sleep(0.1)  # Ensure the message is sent before returning

        return TriggerResponse(success=True, message="Chirp stopped and zero message published.")

    def publish_loop(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            with self.lock:
                if self.running and self.start_time is not None:
                    t = (rospy.Time.now() - self.start_time).to_sec()
                    f_t = self.alpha * t

                    ## Saturation of the Frequency
                    if f_t > self.rate_hz:
                        f_t = self.rate_hz/4.0
                        rospy.logwarn("Frequency saturated at %f Hz", f_t)

                    msg = Float32MultiArray()
                    msg.data = [self.amplitude * np.abs(np.sin(2 * np.pi * f_t * t + i * 2 * np.pi / N_MOTORS)) for i in range(N_MOTORS)]
                    # msg.data = [self.amplitude * np.abs(np.sin(2 * np.pi * f_t * t + i * 2 * np.pi / N_MOTORS)) for i in range(N_MOTORS)] + [0.0] * (7 - N_MOTORS)

                    self.pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('chirp_publisher_node')
    ChirpPublisher()
    rospy.loginfo("Chirp publisher node started.")
    rospy.spin()