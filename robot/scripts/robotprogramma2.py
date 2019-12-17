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

from io import StringIO
from camera_node.msg import prop
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
button = False
client = None
x = 0
y = 0
depth = 0

FUN_SET_DIGITAL_OUT = 1
FUN_SET_FLAG = 2
FUN_SET_ANALOG_OUT = 3
FUN_SET_TOOL_VOLTAGE = 4

Digital_Out_States = [0,0,0,0,0,0,0,0,0,0]  #8(controller)+2(tool)
Digital_In_States = [0,0,0,0,0,0,0,0,0,0]   #8(controller)+2(tool)

#/*================================ Functies ========================================*/
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
    global button
    if (msg.digital_in_states[1].state == True):
        button = True
        # print(button)
    if (msg.digital_in_states[0].state == True):
        button = False
        # print(button)

def propCallback(msg):
    global x
    global y
    global depth
    global newdata
    x = msg.x
    y = msg.y
    depth = msg.d
    newdata = True
    # print (x)
    # print (y)

def pixelToRobotPos(pixelx, pixely, pixelz):
    # Werkt alleen op camerahoogte 60
    
    #pixelz function variabels
    az = -0.00098842
    bz = 0.58280337
    
    robotz = az * pixelz + bz 
    
    #pixelx function variables
    axa = 1.574876543209876e-06
    axb = 2.1168888888889155e-05
    
    bxa = -0.0005164119135802469
    bxb = -0.037168092222222227
    
    #pixelx functions
    ax = axa * pixelz + axb
    bx = bxa * pixelz + bxb
    
    roboty = ax * pixelx + bx
    
    #pixely function variables
    aya = 1.6231481481481479e-06
    ayb = -1.1866666666665582e-06
    
    bya = -0.0004032479629629631
    byb = 0.6218069633333334
    
    #pixely functions
    ay = aya * pixelz + ayb
    by = bya * pixelz + byb
    
    robotx = ay * pixely + by
    
    return robotx, roboty, robotz

def robotMove(pose):
    v = 0.3
    a = 0.2
    rob.movel(pose, acc=a, vel=v, wait=False)
    time.sleep(0.2)
    while True:
        if rob.is_program_running() == False:
            break


#/*================================ main functie ========================================*/
def main():

    try:
        global newdata
        newdata = False

        tcp = [0, 0, 0.110, 0, 0, 0]
        rob.set_tcp(tcp)
        time.sleep(0.2)

        rospy.init_node('robotcalibratie')
        set_states()
        rospy.Subscriber("/ur_driver/io_states", IOStates, IOStates_callback)
        rospy.Subscriber("/mask_node/prop", prop, propCallback)

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

        # rospy.spin()

        print("Press the start button to start the program and move to the neutral pose")
        neutralpose = [0.300, 0, 0.250, 2.215, -2.215, 0]
        while True:
            if button == True:
                robotMove(neutralpose)
                break

#/*================================ Start programma loop ========================================*/
        print("Waiting for new coordinates")
        while True:
            if newdata == True:
                newdata = False
                xrobot, yrobot, zrobot = pixelToRobotPos(x, y, depth)
                pose1 = [xrobot, yrobot, 0.270, 2.215, -2.215, 0]
                poseobject = [xrobot, yrobot, zrobot, 2.215, -2.215, 0]
                print("Received new coordinates")
                print(poseobject)
                input("Press enter to continue")
                
                robotMove(pose1)
                robotMove(poseobject)

                input("Press enter to move away from object")

                robotMove(pose1)
                robotMove(neutralpose)
                newdata = False
                print("Waiting for new coordinates")
            
            if (button == False):
                print("Stopping program")
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



