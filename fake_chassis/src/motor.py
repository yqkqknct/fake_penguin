#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

Rspeed = 40
Lspeed = 38
Rmotor1 = 37
Rmotor2 = 35
Lmotor1 = 33
Lmotor2 = 31
GPIO.setmode(GPIO.BOARD)
GPIO.setup(Rspeed, GPIO.OUT)
GPIO.setup(Lspeed, GPIO.OUT)
GPIO.setup(Rmotor1, GPIO.OUT)
GPIO.setup(Rmotor2, GPIO.OUT)
GPIO.setup(Lmotor1, GPIO.OUT)
GPIO.setup(Lmotor2, GPIO.OUT)
pR = GPIO.PWM(Rspeed, 50)
pL = GPIO.PWM(Lspeed, 50)
pR.start(50)
pL.start(50)

def RF():
    GPIO.output(Rmotor1, GPIO.HIGH)
    GPIO.output(Rmotor2, GPIO.LOW)
def RB():
    GPIO.output(Rmotor2, GPIO.HIGH)
    GPIO.output(Rmotor1, GPIO.LOW)
def RS():
    GPIO.output(Rmotor1, GPIO.LOW)
    GPIO.output(Rmotor2, GPIO.LOW)

def LF():
    GPIO.output(Lmotor1, GPIO.HIGH)
    GPIO.output(Lmotor2, GPIO.LOW)
def LB():
    GPIO.output(Lmotor2, GPIO.HIGH)
    GPIO.output(Lmotor1, GPIO.LOW)
def LS():
    GPIO.output(Lmotor1, GPIO.LOW)
    GPIO.output(Lmotor2, GPIO.LOW)

def setRspeed(sp):
    pR.ChangeDutyCycle(sp)
def setLspeed(sp):
    pL.ChangeDutyCycle(sp)

def stop():
    RS()
    LS()

def setWay(c):
    if c in [1, 2, 3]:
        LF()
    elif c in [5, 6, 7]:
        LB()
    else:
        LS()

    if c in [1, 7, 8]:
        RF()
    elif c in [3, 4, 5]:
        RB()
    else:
        RS()

def Ldetailcommand(c):
    if c > 100:
        c = 100
    if c < -100:
        c = -100
    setLspeed(abs(c))
    if c > 0:
        LF()
    elif c < 0:
        LB()
    else:
        LS()

def Rdetailcommand(c):
    if c > 100:
        c = 100
    if c < -100:
        c = -100
    setRspeed(abs(c))
    if c > 0:
        RF()
    elif c < 0:
        RB()
    else:
        RS()

motor_speed = 50
motor_command = []
get_command_time = time.time()
def m_cb(data):
    global motor_command, get_command_time, motor_speed
    motor_command = data.data
    get_command_time = time.time()
    if len(motor_command) == 3 and motor_command[0] == 1:
        motor_speed = int(motor_command[2])

rospy.init_node('Fake_motor', anonymous=True)
rospy.Subscriber('/motor', Int32MultiArray, m_cb)
print "init done" 
while not rospy.is_shutdown():
    time.sleep(0.001)
    if time.time()-get_command_time > 2:
        stop()
        continue
    if len(motor_command) < 2:
        continue
    if len(motor_command) == 3:
        if motor_command[0] == 1:
            setRspeed(motor_speed)
            setLspeed(motor_speed)
            setWay(motor_command[1])
        elif motor_command[0] == 2:
            Ldetailcommand(motor_command[1])
            Rdetailcommand(motor_command[2])


pR.stop()
pL.stop()
