#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty, EmptyResponse
import threading
import time

DEFAULT_ALPHA = 0.005
DEFAULT_RATE = 1000.0
DEFAULT_AMPLITUDE = 4.0

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
        self.start_srv = rospy.Service('/start_chirp', Empty, self.start_callback)
        self.stop_srv = rospy.Service('/stop_chirp', Empty, self.stop_callback)

    def start_callback(self, req):
        rospy.loginfo("Chirp start requested. Resetting time after 1 second...")
        with self.lock:
            self.start_time = None  # Will be set inside loop after sleep
            self.running = True
        time.sleep(1.0)
        with self.lock:
            self.start_time = rospy.Time.now()
        return EmptyResponse()

    def stop_callback(self, req):
        rospy.loginfo("Chirp stop requested.")
        with self.lock:
            self.running = False
        return EmptyResponse()

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

                    y = np.sin(2 * np.pi * f_t * t)
                    msg = Float32MultiArray()
                    msg.data = [self.amplitude * np.abs(np.sin(2 * np.pi * f_t * t + i * 2 * np.pi / 7)) for i in range(7)]

                    self.pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('chirp_publisher_node')
    ChirpPublisher()
    rospy.loginfo("Chirp publisher node started.")
    rospy.spin()