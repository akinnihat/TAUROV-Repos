from pymavlink import mavutil
import sys
import time

master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

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
    
def set_rc_channel_pwm(channel_id, pwm=1500):
    
    if channel_id < 1 or channel_id > 11:
        print("Channel does not exist.")
        return
        
    rc_channel_values = [1500 for _ in range(8)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)
    
def resetPWMs():
    for i in range(1, 6):
        set_rc_channel_pwm(i, 1500)
    
disarmAuv()

time.sleep(2)

armAuv()
resetPWMs()
time.sleep(40)

set_rc_channel_pwm(3, 1150) # Throttle

time.sleep(6)

set_rc_channel_pwm(5, 1700) # Forward

time.sleep(11)

set_rc_channel_pwm(5, 1500) # Stop

time.sleep(4)

set_rc_channel_pwm(4, 1300) # Yaw

time.sleep(6)

set_rc_channel_pwm(4, 1700) # Yaw

time.sleep(6)

while True: # Emergency stop button test
    set_rc_channel_pwm(5, 1650)
    
resetPWMs()

disarmAuv()
