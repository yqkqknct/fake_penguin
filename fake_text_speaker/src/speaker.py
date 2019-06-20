#!/usr/bin/env python

import speech_recognition as sr

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

#obtain audio from the microphone

class text_speaker:
    
    def __init__(self):
        pub_name = sys.argv[1] if len(sys.argv) >= 2 else "speech"
         
        self.recognize()
        self.image_sub = rospy.Subscriber('/speaker/text', String, self.callback)

    def recognize(self, text):
        tts = gTTS(text=text, lang='en')
        tts.save('test.wav')


def main():
    # Initialize speech_recognizer class
    rospy.init_node('speech_recognizer', anonymous=True)
    sr = speech_recognizer()
    # init a node
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Bye')

    # some deconstruction?

if __name__ == '__main__':
    main()
