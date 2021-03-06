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
import os

from io import StringIO
from abbp_mask.srv import DepthPoseService
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

#/*================================ main functie ========================================*/
def main():

    try:
        global az, bz, axa, axb, bxa, bxb, aya, ayb, bya, byb
        weightsPath = os.path.dirname(os.path.realpath(__file__)) + "/weights60.csv"
        az, bz, axa, axb, bxa, bxb, aya, ayb, bya, byb = loadWeigths(weightsPath)

        # Tool center point inladen, aanwijspunt is 106 mm lang (4 mm speling)
        tcp = [0, 0, 0.110, 0, 0, 0]
        rob.set_tcp(tcp)
        time.sleep(0.2) # Wanneer er een commando naar de robot gestuurd wordt, moet er een paar tienden van een seconden gewacht worden, zodat de robot het kan verwerken

        rospy.init_node('abbp_robot_node') # Node opstarten
        set_states() # IO states van de robot initialiseren
        rospy.Subscriber("/ur_driver/io_states", IOStates, IOStates_callback) # Subscribe op io states

        # Programma starten
        print("Press the start button to start the program and move to the neutral pose")
        neutralpose = [0.300, 0, 0.250, 2.215, -2.215, 0] # Neutrale positie
        while True:
            if button == True: # Wanneer startknop is ingedrukt, beweeg naar neutrale positie
                robotMove(neutralpose)
                break

#/*================================ Start programma loop ========================================*/
        print("Waiting for new coordinates")
        while True:
            rospy.wait_for_service('/abbp_mask_node/trigger_mask')
            depthServiceHandle = rospy.ServiceProxy('/abbp_mask_node/trigger_mask', DepthPoseService) # Nieuwe foto maken en pixelwaarden van objecten opvragen
            depthServiceResponse = depthServiceHandle()
            if depthServiceResponse.success == False: # Wanneer er geen objecten meer in de bak zitten, stop het programma
                print("No objects present, stopping program")
                break
            depthPose = depthServiceResponse.pose
            xrobot, yrobot, zrobot = pixelToRobotPos(depthPose.x, depthPose.y, depthPose.depth)
            pose1 = [xrobot, yrobot, 0.270, 2.215, -2.215, 0]
            poseobject = [xrobot, yrobot, zrobot, 2.215, -2.215, 0]
            
            print("Received new coordinates")
            print(poseobject)
                
            robotMove(pose1)
            robotMove(poseobject)
            time.sleep(1)

            robotMove(pose1)
            robotMove(neutralpose)
            time.sleep(1)
            print("Waiting for new coordinates")
            
            if (button == False): # Wanneer stopknop is ingedrukt, dan programma afsluiten
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

def pixelToRobotPos(pixelx, pixely, pixelz): # Vertalen pixelwaarden naar robotwaarden

    #pixelz function
    robotz = az * pixelz + bz 
    
    #pixelx functions
    ax = axa * pixelz + axb
    bx = bxa * pixelz + bxb
    
    roboty = ax * pixelx + bx
    
    #pixely functions
    ay = aya * pixelz + ayb
    by = bya * pixelz + byb
    
    robotx = ay * pixely + by
    
    return robotx, roboty, robotz

def robotMove(pose):
    v = 0.8
    a = 0.6
    rob.movel(pose, acc=a, vel=v, wait=False)
    time.sleep(0.2)
    while True:
        if rob.is_program_running() == False: # Wachten tot robot klaar is met beweging
            break

def loadWeigths(weightsPath):
    weights = np.genfromtxt(weightsPath, delimiter=',')
    return weights[0], weights[1], weights[2], weights[3], weights[4], weights[5], weights[6], weights[7], weights[8], weights[9]

if __name__ == '__main__': main()