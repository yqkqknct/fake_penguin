#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from gtts import gTTS
import os,signal
import subprocess
import time

def ave(data):
    if not data or not len(data):
        return 0
    return float(sum(data))/len(data)

class bluetooth:
    
    def __init__(self):
        self.speaker_pub = rospy.Publisher('/speaker/input', String, queue_size = 10)
        rospy.Subscriber('/iBeacon/distance', Float32, self.callback)
        self.count = []

    def callback(self, data):
        d = float(data.data)
        self.count.append(d)
        if ave(count) > 1.5:
            self.speaker_pub.publish("0letitgo.mp3")
        else:
            self.speaker_pub.publish("1h")
        if len(self.count) > 10:
            self.count.pop(0)

def main():
    rospy.init_node('bluetooth', anonymous=True)
    bt = bluetooth()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Bye')

if __name__ == '__main__':
    main()
