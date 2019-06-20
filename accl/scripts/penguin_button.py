#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

button = 40
GPIO.setmode(GPIO.BOARD)
GPIO.setup(button, GPIO.IN)

rospy.init_node('egg_button', anonymous=True)
pub1 = rospy.Publisher('/button/egg', Int32, queue_size = 10)

state = 0

while not rospy.is_shutdown():
    time.sleep(0.001)
    R = GPIO.input(button)
    if state != R:
        if R == 1:
            pub1.publish(1)
        else:
            pub1.publish(0)
    state = R
