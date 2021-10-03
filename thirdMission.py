import numpy as np
import time
from pymavlink import mavutil
import imutils
from brping import Ping1D

master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)
myPing = Ping1D()
myPing.connect_serial("/dev/ttyUSB0", 115200)
if myPing.initialize() is False:
    print("Failed to initialize Ping.")
    
balance = 0

def armAuv():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)
    
def disarmAuv():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
    
def forward(balance):
    print("Balance", balance)
    if balance == 50:
        while True:
            set_rc_channel_pwm(5, 1890)
            balance = balance + 1
            if balance == 150:
                print("Degis")
                balance = 0
                break
    balance = balance + 1

def resetPWMs():
    for i in range(1, 6):
        set_rc_channel_pwm(i, 1500)

def set_rc_channel_pwm(channel_id, pwm=1500):

    rc_channel_values = [1500 for _ in range(8)]

    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)
                   
disarmAuv()
time.sleep(10)
armAuv()
data = myPing.get_distance()
oldDistance = data["distance"]

while True:
    set_rc_channel_pwm(4, 1630)
    try:
        data = myPing.get_distance()
        distance = data["distance"]
        if abs(oldDistance - distance) > 5000:
            break
    except KeyboardInterrupt:
        disarmAuv()
        print("End of Mission")
        break
    print("Distance:", distance)
    print("Distance diff:", abs(oldDistance - distance))
    time.sleep(0.2)
    
forward()
time.sleep(40)