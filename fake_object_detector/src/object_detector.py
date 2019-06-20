#!/usr/bin/env python

# References: https://www.hackster.io/mjrobot/automatic-vision-object-tracking-5575c4

import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String, UInt16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class object_detector:
    
    def __init__(self):
        self.pos_pub = rospy.Publisher('object_location', UInt16MultiArray)
        self.bridge = CvBridge()
        
        sub_ndname = '/camera1/usb_cam1/image_raw'
        if len(sys.argv) > 1 and sys.argv[1] == '1': 
            sub_ndname = '/camera1/usb_cam1/image_raw'
        if len(sys.argv) > 1 and sys.argv[1] == '2': 
            sub_ndname = '/camera2/usb_cam2/image_raw'

        self.image_sub = rospy.Subscriber(sub_ndname, Image, self.callback)

    def callback(self, frame):
        try:
            frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame = cv2.flip(frame, -1)
        
        cv2.imshow('gray', frame)

        img = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_range = np.array([50, 100, 100], dtype=np.uint8)
        upper_range = np.array([90, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_range, upper_range)

        cv2.imshow('mask',mask)
        cv2.imshow('image', img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return
def main():
    rospy.init_node('object_detector', anonymous=True)
    dtr = object_detector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Bye")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
