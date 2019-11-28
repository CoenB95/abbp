#!/usr/bin/env python3

import actionlib
import logging
import math3d as m3d
import roslib
import rospy
import sys
import time
import urx

from camera_node.msg import prop
from control_msgs.msg import FollowJointTrajectoryAction
from math import pi
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
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
    x = msg.x
    y = msg.y
    # print x
    # print y

def kalibreren():
    global xmmperpix
    global ymmperpix

    tcp = [0, 0, 0.106, 0, 0, 0]
    rob.set_tcp(tcp)
    time.sleep(0.2)

    # Coordinatenstelsel aanmaken
    rob.set_freedrive(True, timeout=500)
    print("A new coordinate system will be defined from the next three points")
    print("First point is Origin, second X, third Y")
    
    input("When origin circle is detected press enter")
    origin = (x,y)
    input("Move the robot to origin and press enter")
    pose = rob.getl()
    print("Pixelvalue origin: {}".format(origin))
    print("Robotvalue origin: {}".format(pose[:3]))
    p0 = m3d.Vector(pose[:3])
    
    input("When X circle is detected press enter")
    positivex = (x,y)
    input("Move the robot to X and press enter")
    pose = rob.getl()
    print("Pixelvalue X: {}".format(positivex))
    print("Robotvalue X: {}".format(pose[:3]))
    px = m3d.Vector(pose[:3])
    
    input("When Y circle is detected press enter")
    positivey = (x,y)
    input("Move the robot to Y and press enter")
    pose = rob.getl()
    print("Pixelvalue Y: {}".format(positivey))
    print("Robotvalue Y: {}".format(pose[:3]))
    py = m3d.Vector(pose[:3])

    newPlane = m3d.Transform.new_from_xyp(px - p0, py - p0, p0)
    rob.set_csys(newPlane)
    time.sleep(0.2)
    input("New plane is set! Put the robot in proper place and press enter")
    rob.set_freedrive(False)
# /////////////////////// AFSTAND PER PIXEL NOG BEPALEN!!!!!!!!!!!!!!!////////////////////////////
    xrealworld = px[1] - p0[1]
    yrealworld = py[0] - p0[0]
    xresolution = 640
    yresolution = 480
    xmmperpix = xrealworld/xresolution
    ymmperpix = yrealworld/yresolution

def oudekalibratie():
    global xmmperpix
    global ymmperpix

    origin = [0.4120224728754105, -0.22691786947077103, 0.0023989833706220126, 1.9156710830715367, -2.4346068376538126, 0]
    xpositief = [0.4128422172311459, 0.22647243766728514, 0.0024593076828873656, 2.8605895655882057, -1.2148532445977118, 0]
    ypositief = [0.815116143931001, -0.22671715766873157, 0.00328736800204138, 2.0404649983028413, -2.313233757936987, 0]
    tcp = [0, 0, 0.106, 0, 0, 0]
    rob.set_tcp(tcp)
    time.sleep(0.2)

    orig = m3d.Vector(origin[:3])
    posx = m3d.Vector(xpositief[:3])
    posy = m3d.Vector(ypositief[:3])
    oldPlane = m3d.Transform.new_from_xyp(posx - orig, posy - orig, orig)
    rob.set_csys(oldPlane)
    time.sleep(0.2)

    xrealworld = posx[1] - orig[1]
    yrealworld = posy[0] - orig[0]
    xresolution = 640
    yresolution = 480
    xmmperpix = xrealworld/xresolution
    ymmperpix = yrealworld/yresolution

def pointcloudCallback(msg):
    global pointcloud
    pointcloud = msg.data

def getPixelDepth(x, y):
    global p

def main():

    try:
        rospy.init_node('robotcalibratie')
        set_states()
        rospy.Subscriber("/ur_driver/io_states", IOStates, IOStates_callback)
        rospy.Subscriber("/blob_properties", prop, propCallback)
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, pointcloudCallback)

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
       
        print(knop)
        kalibratie = input("Do you want to calibrate the robot and camera? y/n: ")[0]
        if (kalibratie == 'y'):
            kalibreren()
        else:
            oudekalibratie()

        xCirkel = x
        yCirkel = y
        xRobot = xCirkel * xmmperpix
        yRobot = yCirkel * ymmperpix
        print("x Coördinaat:")
        print(xRobot)
        print("y Coördinaat:")
        print(yRobot)
        poseCirkel = [xRobot, yRobot, -0.100, 0, 0, 3.7]
        # pose1 = [0.27491, 0.20474, 0.12119, 3.14, 0, 0]
        # pose2 = [-0.34810878976703047, -0.01692581365556508, 0.18651047378839763, 3.14, 0, 0]
        # pose3 = [0.78021, -0.12499, 0.29962, 3.14, 0, 0]
        # pose4 = [0, 0, 0, 3.14, 0, 0]
        v = 0.3
        a = 0.2
        # rospy.spin()

        while (True):
            if (knop == True):
                print("knop is in")
            while (knop == True):
                rob.movel(poseCirkel, acc=a, vel=v, wait=False)
                time.sleep(0.3)
                while True:
                    if not rob.is_program_running():
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



