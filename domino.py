import argparse
import struct
import sys
import copy
import time

import rospy
import rospkg

import tf

from gazebo_msgs.srv import (SpawnModel, DeleteModel)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import (Header, Empty)

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)

import baxter_interface
import csv
import listener
import bezier_interpolation
import simulation

from sensor_msgs.msg import Range

def setup_listener_left():
    rospy.Rate(10.0)
    rospy.Subscriber("/robot/range/left_hand_range/state",Range,callback,queue_size = 1)

def callback_left(msg):
    global left_arm_range
    left_arm_range = msg.range

def callback_right(msg):
    global right_arm_range
    right_arm_range = msg.range

def move(arm,x,y,z,r,p,ya,pnp):
    #Move to a given point using cartesian coordinates
    quat = tf.transformations.quaternion_from_euler(r,p,ya)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    #print("x = {0}, y = {1}, z = {2}, roll = {3}, pitch = {4}, yaw = {5}".format(x,y,z,r,p,ya))

    if arm == 'l':
        pnp._servo_to_pose(pose)
    elif arm == 'r':
        print("RIGHT ARM")
        pnp._servo_to_pose(pose)
    return x,y,z,r,p,ya

def safe_point_r(pnp):
    #Function to return arms to tucked position
    right_joint_angles = {'right_s0': -0.2823,
                  'right_s1': -1.13965,
                  'right_e0': 1.0771,
                  'right_e1': 1.08657,
                  'right_w0': -0.387,
                  'right_w1': 1.8194,
                  'right_w2': -1.7079
                  }
    pnp._guarded_move_to_joint_position(right_joint_angles)
    print ("Moving to safe point")

def safe_point_l(pnp):
    #Function to return arms to tucked position    
    left_joint_angles = {'left_s0': -0.5929,
                 'left_s1': -1.3422,
                 'left_e0': 0.3146,
                 'left_e1': 1.3544,
                 'left_w0': 3.059-3.14,
                 'left_w1': 1.5702,
                 'left_w2': -1.072
                 }
    pnp._guarded_move_to_joint_position(left_joint_angles)
    print ("Moving to safe point")

def place(x,y,z,r,p,ya,height,arm,pnp):
    global left_arm_range
    global right_arm_range

    conversion = 0
    # ----------------------
    #rospy.init_node('infra_red_listener')
    rospy.Subscriber("/robot/range/left_hand_range/state",Range,callback_left,queue_size = 1)
    time.sleep(0.5)
    rospy.Subscriber("/robot/range/right_hand_range/state",Range,callback_right,queue_size = 1)
    time.sleep(0.5)
    #rospy.spin()
    # ----------------------
    print(left_arm_range, right_arm_range)
    #Function to hover and place bricks on the table
    if arm == 'l':
        adjustment = left_arm_range * conversion
    else:
        adjustment = right_arm_range * conversion

    move(arm,x,y,z+height,r,p,ya,pnp)
    move(arm,x,y,z,r + adjustment,p,ya,pnp)
    if arm == 'l':
        pnp.gripper_open()
        rospy.sleep(0.1)
    if arm == 'r':
        pnp.gripper_open()
        rospy.sleep(0.1)
    move(arm,x,y,z+height,r,p,ya,pnp)
    #Move back to a safe point
    if arm == 'l':
        brick_place('l',-0.5929,-1.3422,0.3146,1.3544,3.059-3.14,1.5702,-1.072,pnp)
    if arm == 'r':
        brick_place('r',-0.2823,-1.13965,1.0771,1.08657,-0.387,1.8194,-1.7079,pnp)
    return x,y,z+height,r,p,ya

def pick(arm,pnp,movement_count):
    #Function to pick up a brick with a given arm from a set position
    if arm == 'l':
        print ("Picking with left arm")
        #Move to 0.5,0.8,0.5,0,3.14/2,0
        brick_place('l',1.045,-1.2174,-0.5546,1.8941,1.5558,-1.2412,-0.9172,pnp)
        pnp.gripper_open()
        coord = move('l',0.6,0.8,0.5,0,3.14/2,0,pnp)

        #simulation.load_brick1()
        spawn_brick(movement_count)

        pnp.gripper_close()
        brick_place('l',1.045,-1.2174,-0.5546,1.8941,1.5558,-1.2412,-0.9172,pnp)
        # #Move to 0.6,0.5,0.4,0,3.14,-0
        #brick_place('l',-0.5967,-1.344,0.3188,1.3571,3.059,-1.5673,-2.642)
        #Move to 0.6,0.5,0.4,0,3.14,3.14/2
        #brick_place('l',-0.5929,-1.3422,0.3146,1.3544,3.059,-1.5702,-1.072)
        brick_place('l',-0.5929,-1.3422,0.3146,1.3544,3.059-3.14,1.5702,-1.072,pnp)
    elif arm == 'r':
        print ("Picking with right arm")
        #Move tp 0.5,-0.8,0.5,0,3.14/2,0
        brick_place('r',-1.032,-1.222,0.5439,1.897,-1.5479,-1.239,0.9162,pnp)
        pnp.gripper_open()
        print ("move('r',0.6,-0.8,0.5,0,3.14/2,0,pnp)")
        coord = move('r',0.6,-0.8,0.5,0,3.14/2,0,pnp)
        print("MOVED")

        #simulation.load_brick2()
        spawn_brick(movement_count)

        pnp.gripper_close()
        brick_place('r',-1.032,-1.222,0.5439,1.897,-1.5479,-1.239,0.9162,pnp)
        #Move to 0.6,-0.5,0.45,0,3.14,-0
        brick_place('r',-0.1652,-1.2395,0.81048,1.1156,-0.2439,1.7843,-0.222,pnp)
        #Move to 0.6,-0.5,0.45,0,3.14,3.14/2
        brick_place('r',-0.2823,-1.13965,1.0771,1.08657,-0.387,1.8194,-1.7079,pnp)

def pickandplace(arm,pnp,x,y,z,r,p,ya,height,movement_count):
    #Combined pick and place functions
    pick(arm,pnp,movement_count)
    place(x,y,z,r,p,ya,height,arm,pnp)


def brick_place(arm,s0,s1,e0,e1,w0,w1,w2,pnp):
    #Function to use joint angles to move to point
    if arm == 'l':
        joint_angles = {'left_s0': s0,
                         'left_s1': s1,
                         'left_e0': e0,
                         'left_e1': e1,
                         'left_w0': w0,
                         'left_w1': w1,
                         'left_w2': w2
                         }
        pnp._guarded_move_to_joint_position(joint_angles)
    elif arm == 'r':
        joint_angles = {'right_s0': s0,
                         'right_s1': s1,
                         'right_e0': e0,
                         'right_e1': e1,
                         'right_w0': w0,
                         'right_w1': w1,
                         'right_w2': w2
                         }
        pnp._guarded_move_to_joint_position(joint_angles)

def knock_down(x,y,z):
    #Function to knock down the arranged bricks
    if y >= 0:
        print ("Knocking down with left arm")
        arm = 'l'
        offset = 0.1
        brick_place('l',-0.5929,-1.3422,0.3146,1.3544,3.059-3.14,1.5702,-1.072)
        left_pnp.gripper_close()
    elif y <= 0:
        print ("Knocking down with right arm")
        arm = 'r'
        offset = -0.1
        brick_place('r',-0.2823,-1.13965,1.0771,1.08657,-0.387,1.8194,-1.7079)
        right_pnp.gripper_close()
    coord = move(arm,x,y+offset,z,0,3.14,0)
    coord = move(arm,x,y-offset,z,0,3.14,0)

def ik_test(x,y,z,r,p,ya,hover,pnp1,pnp2):
    #Function to test whether a range of coordinates are within workspace
    quat = tf.transformations.quaternion_from_euler(r,p,ya)
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    pose2 = Pose()
    pose2.position.x = x
    pose2.position.y = y
    pose2.position.z = z + hover
    pose2.orientation.x = quat[0]
    pose2.orientation.y = quat[1]
    pose2.orientation.z = quat[2]
    pose2.orientation.w = quat[3]

    # print(x, y, z)

    if y <= 0:
        limb_joints = pnp1.ik_request(pose)
        limb_joints_up = pnp1.ik_request(pose2)
        if limb_joints == False or limb_joints_up == False:
            print ("Right arm failed")
            limb_joints = pnp2.ik_request(pose)
            limb_joints_up = pnp2.ik_request(pose2)
            if limb_joints_up == False or limb_joints_up == False:
                print ("Try 2 - left failed")
                print ("FAILED COORDINATE")
            else:
                print ("SUCCESSFUL COORDINATE")
                pass
        else:
            print ("SUCCESSFUL COORDINATE")
            pass
    elif y >= 0:
        limb_joints = pnp2.ik_request(pose)
        limb_joints_up = pnp2.ik_request(pose2)
        if limb_joints == False or limb_joints_up == False:
            print ("Left arm failed")
            limb_joints = pnp1.ik_request(pose)
            limb_joints_up = pnp1.ik_request(pose2)
            if limb_joints == False or limb_joints_up == False:
                print ("Try 2 - right failed")
                print ("FAILED COORDINATE")
            else:
                print ("SUCCESSFUL COORDINATE")
                pass
        else:
            print ("SUCCESSFUL COORDINATE")
            pass

    return limb_joints,limb_joints_up

def spawn_brick(movement_count):
    if movement_count == 1:
        simulation.load_brick1()
    elif movement_count == 2:
        simulation.load_brick2()
    elif movement_count == 3:
        simulation.load_brick3()
    elif movement_count == 4:
        simulation.load_brick4()
    elif movement_count == 5:
        simulation.load_brick5()
    elif movement_count == 6:
        simulation.load_brick6()
    elif movement_count == 7:
        simulation.load_brick7()
    elif movement_count == 8:
        simulation.load_brick8()