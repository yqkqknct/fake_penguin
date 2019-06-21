#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from gtts import gTTS
import os,signal
import subprocess
import time

file_path = '/home/pi/catkin_ws/src/fake_penguin/fake_speaker/src'
distance_thr = 1

class speaker:

    def __init__(self):
        rospy.Subscriber('/speaker/input', String, self.callback)

        self.player = None

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'egg/speaker heard %s', data.data)
        if data.data != "None":
            # data startswith 0, start playing music
            if data.data[0] == '0':
                # If some player not closed yet, close it.
                if self.player:
                    os.killpg(os.getpgid(self.player.pid), signal.SIGTERM)
                    self.player = None

                filename = data.data[1:]
                self.player = subprocess.Popen("mpg123 " + file_path + "/" + filename, 
                    shell = True, preexec_fn=os.setsid)
                os.killpg(os.getpgid(self.player.pid), signal.SIGTERM)
            
            # data startswith 1, stop playing music.
            elif data.data[0] == '1':
                # if already close, skip it.
                if not self.player:
                    return
                os.killpg(os.getpgid(self.player.pid), signal.SIGTERM)
                self.player = None
            
            # otherwises, convert text to music and play
            else:
                tts = gTTS(text=data.data, lang='en')
                tts.save(file_path + '/test.mp3')
                os.system("mpg123 " + file_path + "/test.mp3")
                os.system("rm -f " + file_path + "/test.mp3")

def main():
    rospy.init_node('speaker', anonymous=True)
    spk = speaker()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Bye')

if __name__ == '__main__':
    main()
