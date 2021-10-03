import cv2
import numpy as np
import time
from pymavlink import mavutil
import imutils
import threading
from brping import Ping1D

xC = False
foundSubmarine = False
endOfMission = False
directionCounter = 1
x, radius = 0, 0
exit_event_detect = threading.Event()
exit_event_search = threading.Event()
exit_event_horizontal = threading.Event()

mtx = [303.26103924, 0, 330.07840914,
       0, 302.17815631, 241.76844028,
       0,     0     ,       1        ]

mtx = np.array(mtx)
mtx = mtx.reshape((3,3))

dist = [-0.33920992, 0.12270265, 0.00046082, -0.00203071, -0.02049525]

dist = np.array(dist)
dist = dist.reshape((1,5))

low_red = np.array([161, 155, 84])
high_red = np.array([179, 255, 255])

class auvThreads(threading.Thread):
    def __init__(self, x, radius, foundSubmarine, endOfMission, directionCounter, xC):
        threading.Thread.__init__(self)
        self.x = x
        self.radius = radius
        self.foundSubmarine = foundSubmarine
        self.endOfMission = endOfMission
        self.directionCounter = directionCounter
        self.xC = xC
        self.lock = threading.Lock()
        self.event_found_submarine = threading.Event()
        detect = threading.Thread(target=self.detectSubmarine, args=(self.event_found_submarine,))
        search = threading.Thread(target=self.searchSubmarine, args=(self.event_found_submarine,))
        horizontal = threading.Thread(target=self.horizontal, args=(self.event_found_submarine,))
        detect.start()
        search.start()
        horizontal.start()
        detect.join()
        search.join()
        horizontal.join()

    def detectSubmarine(self, event_found_submarine):
        while endOfMission == False:
            
            if exit_event_detect.is_set():
                break
            
            _, frame = cap.read()
            frame = cv2.resize(frame, (640, 480))
            frame = imutils.rotate(frame, angle=180)

            newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640,480), 1, (640,480))
            dst = cv2.undistort(frame, mtx, dist, newcameramtx)
            #dst = cv2.addWeighted(dst, 1, np.zeros(dst.shape, dst.dtype), 0, 16)
            
            hsv_frame = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)

            red_mask = cv2.inRange(hsv_frame, low_red, high_red)
            red = cv2.bitwise_and(dst, dst, mask = red_mask)

            frameCanny = cv2.Canny(red, 100, 230) 
            kernel = np.ones((5, 5))
            frameDilate = cv2.dilate(frameCanny, kernel, iterations=2)
            
            try:
                self.x, self.radius = getContours(frameDilate, red)
                print("Info:", self.x, self.radius)
                self.event_found_submarine.set()
            
            except:
                self.x, radius = 0, 0
                self.foundSubmarine = False
                print("0, 0")
                try:                    
                    self.event_found_submarine.clear()
                    self.lock.release()
                except: pass
                
            #v2.imshow("red", red)
            #out.write(red)

            key = cv2.waitKey(1)
            if key == 27:
                break

        self.endOfMission = True
        out.release()
        cap.release()
        disarmAuv()

        cv2.destroyAllWindows()
        
        exit_event_search.set()
        exit_event_horizontal.set()
        exit_event_detect.set()
        
    
    def searchSubmarine(self, event_found_submarine):
        
        while endOfMission == False:

            if not self.event_found_submarine.is_set():
                time.sleep(0.25)
                if not self.event_found_submarine.is_set():
                    if self.directionCounter == 0:
                        self.directionCounter = 1
                        for i in range(5):
                            sonarSensor(self.directionCounter)
                            time.sleep(0.15)
                            if not self.event_found_submarine.is_set():
                                set_rc_channel_pwm(4, 1620)
                                print("Go To Right: Search")
                            else: break
                            
                        for i in range(10):
                            sonarSensor(self.directionCounter)
                            time.sleep(0.15)
                            if not self.event_found_submarine.is_set():
                                set_rc_channel_pwm(5, 1720)
                            else: break   
                        
                    elif self.directionCounter == 1:
                        self.directionCounter = 0
                        for i in range(10):
                            sonarSensor(self.directionCounter)
                            time.sleep(0.15)
                            if not self.event_found_submarine.is_set():
                                set_rc_channel_pwm(4, 1370)
                                print("Go To Left: Search")
                            else: break
                            
                        for i in range(10):
                            sonarSensor(self.directionCounter)
                            time.sleep(0.15)    
                            if not self.event_found_submarine.is_set():
                                set_rc_channel_pwm(5, 1720)
                            else: break   
                    else: pass

                    if exit_event_search.is_set():
                        print("Locked On Target")
                        break
                    else: self.lock.acquire()
                else: pass                
            
    def horizontal(self, event_found_submarine):
        while endOfMission == False:
            time.sleep(0.01)
            if self.event_found_submarine.is_set():
                
                if exit_event_horizontal.is_set():
                    print("Horizontal: Set")
                    break
                    
                if 0 < self.x < 270:
                    set_rc_channel_pwm(6, 1400) 
                    print("Go Left: Target")                               
                elif 270 < self.x < 370:
                    set_rc_channel_pwm(5, 1650)
                    print("Forward: Target")
                elif self.x > 370:
                    set_rc_channel_pwm(6, 1600) 
                    print("Go Right: Target")
                else: pass
                
                if self.radius > 100:
                    set_rc_channel_pwm(3, 1100)
                    time.sleep(3)
                    set_rc_channel_pwm(5, 1650)
                    time.sleep(4)
                    resetPWMs()
                    disarmAuv()
                    self.endOfMission = True
 
def getContours(frame, frameContour):
        contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areaMin = 100
        epsilonValue  = 0.1
        for cnt in contours:
            area = cv2.contourArea(cnt)
            ((xCirc, yCirc), radius) = cv2.minEnclosingCircle(cnt)
            M = cv2.moments(cnt)
            centerX = int(M['m10']/M['m00'])
            centerY = int(M['m01']/M['m00'])
            cv2.circle(frameContour, (int(xCirc), int(yCirc)), int(radius), (0, 255, 0))
            cv2.circle(frameContour, (centerX, centerY), 2, (0, 255, 0), 4) 
            cv2.putText(frameContour, "Submarine", (centerX, centerY), cv2.FONT_HERSHEY_COMPLEX, .7,
                (255, 255, 0), 1)

            coordinateList = [centerX, radius]
            return coordinateList
            
# -------------------------------------------

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
    
def sonarSensor(direction):
    data = myPing.get_distance()
    if data:
        print("Distance: %s\tConfidence: %s%%" % (data["distance"], data["confidence"]))
    else:
        print("Failed to get distance data.")   
    if data["distance"] <= 1000:
        resetPWMs()
        while True:
            data = myPing.get_distance()
            if direction == 0:
                set_rc_channel_pwm(4, 1620)
            elif direction == 1:
                set_rc_channel_pwm(4, 1480)
            if data["distance"] > 5000:
               break
               
# -------------------------------------------

cap = cv2.VideoCapture(0) 
fourcc = cv2.VideoWriter_fourcc('D','I','V','X')
out = cv2.VideoWriter('uwOutput17.avi', fourcc, 30, (640, 480), isColor=True)
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)
myPing = Ping1D()
myPing.connect_serial("/dev/ttyUSB0", 115200)
if myPing.initialize() is False:
    print("Failed to initialize Ping.")

disarmAuv()
time.sleep(35)
armAuv()
executeMission = auvThreads(x, radius, foundSubmarine, endOfMission, directionCounter, xC)
executeMission.start()
