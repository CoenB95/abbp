#!/usr/bin/env python3

import actionlib
import logging
import math3d as m3d
import roslib
import rospy
import sys
import time
import urx
import numpy as np
import csv

from abbp_mask.msg import DepthPose
from control_msgs.msg import FollowJointTrajectoryAction
from math import pi
from math import atan
from math import tan
from sensor_msgs.msg import JointState
from std_msgs.msg import Header,Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO


roslib.load_manifest('ur_driver')

rob = urx.Robot("172.22.22.2")
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

knop = False
client = None
nieuwelaag = False
# origin = [0.36442, 0.20452, 0.10149, 2.4984, -1.9328, 0]
# xpositief = [0.36447, -0.20380, 0.10122, 1.2306, -2.8976, 0]
# ypositief = [0.85440, 0.20367, 0.09968, 2.3768, -2.1229, 0]
# p0 = m3d.Vector(origin[:3])
# px = m3d.Vector(xpositief[:3])
# py = m3d.Vector(ypositief[:3])

# newPlane = m3d.Transform.new_from_xyp(px - p0, py - p0, p0)
# rob.set_csys(newPlane)
# time.sleep(0.3)
# input("New plane is set! press enter")

FUN_SET_DIGITAL_OUT = 1
FUN_SET_FLAG = 2
FUN_SET_ANALOG_OUT = 3
FUN_SET_TOOL_VOLTAGE = 4

#Flag_States = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
Digital_Out_States = [0,0,0,0,0,0,0,0,0,0]  #8(controller)+2(tool)
Digital_In_States = [0,0,0,0,0,0,0,0,0,0]   #8(controller)+2(tool)
Analog_Out_States = [0,0]  #2(controller)
Analog_In_States = [0,0]   #2(controller)+0(tool)

ANALOG_TOLERANCE_VALUE = 0.01

def set_digital_out(pin, val):
    try:
        set_io(FUN_SET_DIGITAL_OUT, pin, val)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
def set_states():
    rospy.wait_for_service('/ur_driver/set_io')
    global set_io
    set_io = rospy.ServiceProxy('/ur_driver/set_io', SetIO)


def IOStates_callback(msg):
    global knop
    if (msg.digital_in_states[1].state == True):
        knop = True
        # print(knop)
    if (msg.digital_in_states[0].state == True):
        knop = False
        # print(knop)

def propCallback(msg):
    global x
    global y
    global diepte
    x = msg.x
    y = msg.y
    diepte = msg.depth
    # print x
    # print y

def saveValues(tabel):
    global nieuwelaag
    global laag
    robotpos = rob.getl()
    robotwaardes = robotpos[:3]
    pixelwaardes = [x, y, diepte]
    np.set_printoptions(suppress=True)
    if nieuwelaag == True:
        laag += 1
    robotpixel = np.append(pixelwaardes, robotwaardes)
    robotpixel = np.hstack((robotpixel, laag))
    tabel = np.vstack([tabel, robotpixel])
    return tabel


def main():

    try:
        tcp = [0, -0.100, 0, 0, 0, 0]
        rob.set_tcp(tcp)
        time.sleep(0.2)
        global xopschuif
        global yopschuif
        global zopschuif
        global nieuwelaag
        global laag

        nieuwelaag = False
        laag = 1
        xopschuif = 0.02 # 5 mm opschuiven
        yopschuif = 0.02 # 5 mm opschuiven
        zopschuif = 0.02 # 5 mm opschuiven
        rospy.init_node('robotcalibratie')
        set_states()
        rospy.Subscriber("/ur_driver/io_states", IOStates, IOStates_callback)
        rospy.Subscriber("/abbp_mask_node/circle/pose", DepthPose, propCallback)

        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Connected to server")
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name

        v = 0.3
        a = 0.2
        # rospy.spin()
        
        tabel = np.zeros(shape=(1,7))
        input("Press the start button to start and then press enter")
        currentpose = rob.getl()
        if x > 0 and y > 0 and diepte >0:
            tabel = saveValues(tabel)
        middelpuntX = currentpose[0]
        middelpuntY = currentpose[1]
        startpose = [0.471, 0.160, 0.035, currentpose[3], currentpose[4], currentpose[5]]
        nieuwelaag = True

        while True: # Opschuiven in de x richting van het robotassenstelsel
            if nieuwelaag == True:
                startpose[2] = currentpose[2]
                rob.movel(startpose, acc=a, vel=v, wait=False)
                time.sleep(0.2)
                while True:
                    if rob.is_program_running() == False:
                        break
                currentpose[0] = startpose[0]
                currentpose[1] = startpose[1]
                nieuwelaag =  False
                if x > 0 and y > 0 and diepte >0:
                    tabel = saveValues(tabel)


            currentpose[0] += xopschuif
            rob.movel(currentpose, acc=a, vel=v, wait=False)
            time.sleep(0.2)
            while True:
                if rob.is_program_running() == False:
                    break

            if x > 0 and y > 0 and diepte >0:
                tabel = saveValues(tabel)

            if currentpose[0] < 0.470 or currentpose[0] > 0.740: # Wanneer een hele rij is afgelegd, in de Y een beetje opschuiven en achteruit bewgen in X
                currentpose[1] -= yopschuif
                rob.movel(currentpose, acc=a, vel=v, wait=False)
                time.sleep(0.2)
                while True:
                    if rob.is_program_running() == False:
                        break

                if x > 0 and y > 0 and diepte > 0:
                    tabel = saveValues(tabel)

                if currentpose[1] < -0.160 or currentpose[1] > 0.161:
                    nieuwelaag = True
                    currentpose[0] = middelpuntX
                    currentpose[1] = middelpuntY
                    currentpose[2] += zopschuif
                    # yopschuif = yopschuif * -1 # yopschuif omklappen zodat hij weer de andere kant op gaat
                    rob.movel(currentpose, acc=a, vel=v, wait=False)
                    time.sleep(0.2)
                    while True:
                        if rob.is_program_running() == False:
                            break
                    print(tabel)
                    if x > 0 and y > 0 and diepte > 0:
                        tabel = saveValues(tabel)
                
                if nieuwelaag == True:
                    xopschuif = 0.02
                else:
                    xopschuif = xopschuif * -1 # xopschuif omklappen zodat hij weer de andere kant op gaat
            if (knop == False) or currentpose[2] > 0.2:
                np.savetxt('tabel50.csv', tabel, '%19.6f', delimiter= ',')
                break
                
            

    except rospy.ROSInterruptException:
        print("ERROR: ROS interrupted!")
        pass
    except Exception as e:
        print("CRITICAL ERROR: %s" %e)
    finally:
        rob.close()
        sys.exit()

if __name__ == '__main__': main()



