#!/usr/bin/env python

import speech_recognition as sr

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

#obtain audio from the microphone

class speech_recognizer:
    
    def __init__(self):
        pub_name = sys.argv[1] if len(sys.argv) >= 2 else "speech"
        self.pub = rospy.Publisher(pub_name, String)
        
        self.recognize()

    def recognize(self):
        r=sr.Recognizer()
        try: 
            while True:
                with sr.Microphone() as source:
                    print("Please wait. Calibrating microphone...")
                    #listen for 1 seconds and create the ambient noise energy level
                    r.adjust_for_ambient_noise(source, duration=0.5)
                    print("Say something!")
                    audio=r.listen(source)
                    # recognize speech using Google Speech Recognition

                try:
                    print("Google Speech Recognition thinks you said:")
                    ret = r.recognize_google(audio)
                    text_list = ret.split(' ')
                    
                    for text in text_list:
                        self.pub.publish(text)
                        print('Publish ' + text)

                except sr.UnknownValueError:
                    print("Google Speech Recognition could not understand audio")
                except sr.RequestError as e:
                    print("No response from Google Speech Recognition service: {0}".format(e))
        except KeyboardInterrupt:
            print('Stopped')

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
