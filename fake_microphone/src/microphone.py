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
import speech_recognition as sr
import os
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32

class microphone:
    
    def __init__(self):
        rospy.Subscriber('/button/egg', Int32, self.callback)
        self.pub = rospy.Publisher('/microphone/output', String, queue_size=10)

    # Microphone was triggered, start recording and recognizing
    def callback(self, data):
        # dont trigger
        if data.data == 0:
            return
        r = sr.Recognizer()
        with sr.Microphone() as source:
            rospy.loginfo("Please wait. Calibrating microphone...")
            r.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Say something!")
            audio = r.listen(source, timeout=10)
        try:
            rospy.loginfo("Google Speech Recognition thinks you said:")
            
            ret = r.recognize_google(audio)
            rospy.loginfo(ret)
            self.pub.publish(ret)

        except sr.UnknownValueError:
            rospy.loginfo("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            rospy.loginfo("No response from Google Speech Recognition service: {0}".format(e))

def main():
    rospy.init_node('microphone', anonymous=True)
    stt = microphone()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Bye')

if __name__ == '__main__':
    main()
