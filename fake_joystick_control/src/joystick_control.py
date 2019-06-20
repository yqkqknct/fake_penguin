#!/usr/bin/env python

import rospy
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int32MultiArray

forward_thr = 50
backward_thr = -30
right_thr = 50
left_thr = -50
def get_pos_cb(data):
    data = data.data
    if data[1] < forward_thr and data[1] > backward_thr:
        if data[0] > right_thr:
            pub_data = Int32MultiArray(data = [1, 3, 50])
            pub1.publish(pub_data)
        elif data[0] < left_thr:
            pub_data = Int32MultiArray(data = [1, 7, 50])
            pub1.publish(pub_data)
        else:
            pub_data = Int32MultiArray(data = [1, 0, 50])
            pub1.publish(pub_data)
    else:
        if data[1] > forward_thr:
            pub_data = Int32MultiArray(data = [1, 1, 50])
            pub1.publish(pub_data)
        elif data[1] < backward_thr:
            pub_data = Int32MultiArray(data = [1, 5, 50])
            pub1.publish(pub_data)


rospy.init_node('joy_control', anonymous=True)
rospy.Subscriber("/egg/angle", Floats, get_pos_cb)
pub1 = rospy.Publisher("/motor", Int32MultiArray, queue_size = 10)

while not rospy.is_shutdown():
    pass
