#!/usr/bin/env python 
PKG = 'accl'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy_tutorials.msg import Floats

def callback(data):
    print rospy.get_name(), "I heard %s"%str(data.data)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("chatter", Floats, callback)
    rospy.spin()
if __name__ == '__main__':
    listener()
