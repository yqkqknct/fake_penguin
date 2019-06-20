#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

Lbutton = 16
Rbutton = 18
GPIO.setmode(GPIO.BOARD)
GPIO.setup(Rbutton, GPIO.IN)
GPIO.setup(Lbutton, GPIO.IN)

rospy.init_node('penguin_button', anonymous=True)
pub1 = rospy.Publisher('/button/penguin/left', Int32, queue_size = 10)
pub2 = rospy.Publisher('/button/penguin/right', Int32, queue_size = 10)
print "init done" 

Rstate = 0
Lstate = 0

while not rospy.is_shutdown():
    time.sleep(0.001)
    R = GPIO.input(Rbutton)
    L = GPIO.input(Lbutton)
    if Rstate != R:
        if R == 1:
            pub1.publish(1)
        else:
            pub1.publish(0)
    if Lstate != L:
        if L == 1:
            pub2.publish(1)
        else:
            pub2.publish(0)
    Rstate = R
    Lstate = L
