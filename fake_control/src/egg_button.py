#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Int32

class egg_button:
    
    def __init__(self):
        self.button = 40
        self.state = 0
        self.pub = rospy.Publisher('/button/egg', Int32, queue_size = 10)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.button, GPIO.IN)
        
        self.busyloop()

    def busyloop(self):
        while not rospy.is_shutdown():
            time.sleep(0.001)
            R = GPIO.input(self.button)
            if self.state != R:
                if R == 1:
                    self.pub.publish(1)
                else:
                    self.pub.publish(0)
            self.state = R

def main():
    rospy.init_node('egg_button', anonymous=True)
    eb = egg_button()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Bye')

if __name__ == '__main__':
    main()
