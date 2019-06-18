#!/usr/bin/env python
#license removed for brevity


import rospy
from std_msgs.msg import Float32

import blescan
import sys

import bluetooth._bluetooth as bluez


rospy.init_node('iBeacon_distance', anonymous=True)
pub1 = rospy.Publisher('/iBeacon/distance', Float32, queue_size = 10)
device_UUID = '0ab4d57e77124c899c59ab944a5d47b0'

dev_id = 0
try:
    sock = bluez.hci_open_dev(dev_id)
    print "ble thread started"

except:
    print "error accessing bluetooth device..."
    sys.exit(1)

blescan.hci_le_set_scan_parameters(sock)
blescan.hci_enable_le_scan(sock)

def calculateAccuracy(Tx, rssi):
    if rssi == 0:
        return -1
    ratio = float(rssi)/Tx
    if ratio<1:
        return ratio**10
    return 0.89976*(ratio**7.7095)+0.111
while not rospy.is_shutdown():
    returnedList = blescan.parse_events(sock, 1)
    for beacon in returnedList:
        b_data = beacon.split(',')
        UUID = b_data[-5]
        major = int(b_data[-4])
        minor = int(b_data[-3])
        if UUID != device_UUID:
            continue
        Tx = int(b_data[-2])
        rssi = int(b_data[-1])
        distance = calculateAccuracy(Tx, rssi)
        pub1.publish(distance)
        print(Tx, rssi, distance)
