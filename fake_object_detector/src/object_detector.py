#!/usr/bin/env python

# References: https://www.hackster.io/mjrobot/automatic-vision-object-tracking-5575c4

import numpy as np
import sys
import rospy
import cv2
import time
from collections import deque
from std_msgs.msg import String, UInt16MultiArray, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class object_detector:
    
    def __init__(self):
        self.pos_pub = rospy.Publisher('object_location', UInt16MultiArray)
        self.motor_pub = rospy.Publisher('motor', Int32MultiArray)
        self.bridge = CvBridge()
        
        sub_ndname = '/camera1/usb_cam1/image_raw'
        if len(sys.argv) > 1 and sys.argv[1] == '1': 
            sub_ndname = '/camera1/usb_cam1/image_raw'
        if len(sys.argv) > 1 and sys.argv[1] == '2': 
            sub_ndname = '/camera2/usb_cam2/image_raw'
        
        self.image_sub = rospy.Subscriber(sub_ndname, Image, self.callback)
        self.pts = deque(maxlen=20)

        self.no_cnt = 0

    def callback(self, frame):
        try:
            frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        except CvBridgeError as e:
            print(e)
        frame = cv2.flip(frame, -1)
        #print(frame.shape)        
        #cv2.imshow('gray', frame)
        #print('why not show')
        img = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_range = np.array([20, 40, 100], dtype=np.uint8)
        upper_range = np.array([70, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_range, upper_range)

        w = frame.shape[0]
        h = frame.shape[1]
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if radius > 15:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
            self.no_cnt = 0

        if not center:
            self.no_cnt += 1

        print(self.no_cnt)

        self.pts.appendleft(center)
        for i in range(1, len(self.pts)):
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue
            thickness = int(np.sqrt(20 / float(i + 1)) * 2.5)
            cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

        cv2.imshow('mask',mask)
        #cv2.imshow('image', img)
        cv2.imshow('frame', frame)

        self.motor(center)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return

    def motor(self, center):
        speed = 35
        delay = 0.2
        rotate_delay = 0.1
        if not center:
            #if self.no_cnt > 100:
            #    self.motor_pub.publish(Int32MultiArray(data=[1, 3, speed]))
            #    time.sleep(rotate_delay)
            return

        x, y = center
        if x < 300:
            self.motor_pub.publish(Int32MultiArray(data=[1, 7, speed]))
            time.sleep(delay)
            self.motor_pub.publish(Int32MultiArray(data=[1, 0, 0]))
        if x > 340:
            self.motor_pub.publish(Int32MultiArray(data=[1, 3, speed]))
            time.sleep(delay)
            self.motor_pub.publish(Int32MultiArray(data=[1, 0, 0]))


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
