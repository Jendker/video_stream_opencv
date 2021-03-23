#! /usr/bin/env python3
"""
@author: Jedrzej Orbik
"""

import cv2
import rospy
import threading
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Streamer:
    def __init__(self):
        rospy.init_node('dummy_name', anonymous=True)
        self.parse_rosparam()
        self.publisher = rospy.Publisher('image_raw', Image, queue_size=1)
        self.setup_capture_device(exit_on_error=True)
        self.buffer = []
        self._running = False
        self.bridge = CvBridge()
        self.start_capture()

    def __del__(self):
        if hasattr(self, '_thread'):
            self._thread.join()

    def setup_capture_device(self, exit_on_error):
        resource = "/dev/video" + self._video_stream_provider
        if not os.path.exists(resource):
            rospy.logerr("Device %s does not exist.", resource)
            if exit_on_error:
                rospy.signal_shutdown(f"Device {resource} does not exist.")
            return
        rospy.loginfo("Trying to open resource: %s", resource)
        self.cap = cv2.VideoCapture(resource)
        if not self.cap.isOpened() and exit_on_error:
            rospy.logerr(f"Error opening resource: {resource}")
            rospy.loginfo("opencv VideoCapture can't open it")
            rospy.loginfo("The device %s is possibly in use. You could try reconnecting the camera.", resource)
            rospy.signal_shutdown(f"Error opening resource: {resource}")
        if self.cap.isOpened():
            rospy.loginfo("Correctly opened resource.")

    @staticmethod
    def get_param(parameter_name):
        if not rospy.has_param(parameter_name):
            rospy.logerr('Parameter %s not provided. Exiting.', parameter_name)
            exit(1)
        return rospy.get_param(parameter_name)

    def parse_rosparam(self):
        self._video_stream_provider = self.get_param("~video_stream_provider")
        self._buffer_queue_size = self.get_param("~buffer_queue_size")
        self._fps = self.get_param("~fps")
        self._frame_id = self.get_param("~frame_id")
        self._retry_on_fail = self.get_param("~retry_on_fail")

    def start_capture(self):
        self._thread = threading.Thread(target=self.capture)
        self._lock = threading.Lock()
        self._thread.start()

    def capture(self):
        # running at full speed the camera allows
        while not rospy.is_shutdown():
            rval, frame = self.cap.read()
            self._running = rval
            if not rval:
                rospy.logwarn("The frame has not been captured. You could try reconnecting the camera.")
                rospy.sleep(3)
                if self._retry_on_fail:
                    rospy.loginfo("Searching for the device...")
                    self.setup_capture_device(exit_on_error=False)
            else:
                with self._lock:
                    while len(self.buffer) >= self._buffer_queue_size:
                        self.buffer.pop(0)
                    self.buffer.append(frame)

    def publish_image(self, image):
        imgmsg = self.bridge.cv2_to_imgmsg(image)
        imgmsg.header.frame_id = self._frame_id
        self.publisher.publish(imgmsg)

    def run(self):
        rate = rospy.Rate(self._fps)
        while not rospy.is_shutdown():
            image = None
            if self._running:
                with self._lock:
                    if self.buffer:
                        image = self.buffer[-1].copy()
                if image is not None:
                    self.publish_image(image)
            rate.sleep()

def main():
    streamer = Streamer()
    streamer.run()

if __name__ == '__main__':
    main()
