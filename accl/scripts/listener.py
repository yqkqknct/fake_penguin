#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import speech_recognition as sr
import os
pub = None
flag = 0
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'egg/microphone heard %s', data.data)
    global flag 
    if data.data != 1:
        pass
    elif flag == 0:
        print("start speech to text")
        mic_str = speech_text()
        #pub = rospy.Publisher('mic_str', String, queue_size=10)
        global pub
        pub.publish(str(mic_str))

def speech_text():
    global flag
    flag = 1
    r=sr.Recognizer()
    with sr.Microphone() as source:
        print("Please wait. Calibrating microphone...")
        #listen for 1 seconds and create the ambient noise energy level
        r.adjust_for_ambient_noise(source, duration=1)
        rospy.loginfo("Say something!")
        audio=r.listen(source,timeout=10)
    # recognize speech using Google Speech Recognition
    try:
        rospy.loginfo("Google Speech Recognition thinks you said:")
        print(r.recognize_google(audio))
        flag = 0
        return r.recognize_google(audio)
    except sr.UnknownValueError:
        rospy.loginfo("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        rospy.loginfo("No response from Google Speech Recognition service: {0}".format(e))
    flag = 0
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('microphone', anonymous=True)

    rospy.Subscriber('/button/egg', Int32, callback)
    global pub
    pub = rospy.Publisher('/mic/output', String, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
