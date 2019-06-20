#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from gtts import gTTS
import os,signal
import subprocess
import time

file_path = '/home/pi/catkin_ws/src/accl/scripts'
distance_thr = 1


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'egg/speaker heard %s', data.data)
    if data.data != "None":
        if data.data[0] == '0':
            #play file audio
            print("file audio")
            #os.system("mpg123 letitgo.mp3")
            t_end = time.time() + 10
            sub_proc = subprocess.Popen("mpg123 "+file_path+"/letitgo.mp3", shell = True, preexec_fn=os.setsid)
            print('pid: ' + str(sub_proc.pid))
            time.sleep(10)
            #while time.time() < t_end:
            #    pass
            os.killpg(os.getpgid(sub_proc.pid), signal.SIGTERM)
            #os.system("kill -9 "+str(sub_proc.pid+1))
            #os.kill(sub_proc.pid, signal.SIGKILL)
            
        else:
            print("google audio")
            tts = gTTS(text=data.data, lang='en')
            tts.save(file_path+'/test.mp3')
            os.system("mpg123 "+file_path+"/test.mp3")
            os.system("rm -f "+file_path+"/test.mp3")
count = 0
kill_id = 0
sing_flag = 0
def distance(data):
    global count, kill_id, sing_flag
    d = float(data.data)
    print(data.data, count)
    if data.data > distance_thr:
        count = count+1
    else:
        count = 0
        if sing_flag == 1:
            os.killpg(os.getpgid(kill_id), signal.SIGTERM)
            #os.system("kill -9 "+str(kill_id))
            sing_flag = 0
    if count > 2 and sing_flag == 0:
        sub_proc = subprocess.Popen("mpg123 "+file_path+"/letitgo.mp3", shell = True, preexec_fn=os.setsid)
        kill_id = sub_proc.pid
        sing_flag = 1
        #print('pid: ' + str(sub_proc.pid))
        #while time.time() < t_end:
        #    pass


def listener():
    rospy.init_node('speaker', anonymous=True)
    rospy.Subscriber('/speaker/input', String, callback)
    rospy.Subscriber('/iBeacon/distance', Float32,  distance)
    rospy.spin()
if __name__ == '__main__':
    listener()
