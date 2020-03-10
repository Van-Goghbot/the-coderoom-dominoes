import argparse
import struct
import sys
import copy

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
import domino
import subprocess

def load_table(table_pose=Pose(position=Point(x=1.2, y=0.0, z=0.0)),
                       table_reference_frame="world"):
    # Load Table SDF
    table_xml = ''
    with open ("models/table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_brick1(block1_pose=Pose(position=Point(x=0.65, y=0.8, z=1.13)),
                       block1_reference_frame="world"):

    print("LOADING BRICK 1")
    # Load Brick SDF
    block1_xml = ''
    with open ("models/Brick/model.sdf", "r") as block1_file:
        block1_xml=block1_file.read().replace('\n', '')

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick1", block1_xml, "/",
                               block1_pose, block1_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_brick2(block2_pose=Pose(position=Point(x=0.65, y=-0.8, z=1.13)),
                       block2_reference_frame="world"):

    print("LOADING BRICK 2")
    # Load Brick SDF
    block2_xml = ''
    with open ("models/Brick/model.sdf", "r") as block2_file:
        block2_xml=block2_file.read().replace('\n', '')

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick2", block2_xml, "/",
                               block2_pose, block2_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_brick3(block3_pose=Pose(position=Point(x=0.65, y=0.8, z=1.13)),
                       block3_reference_frame="world"):

    print("LOADING BRICK 3")
    # Load Brick SDF
    block3_xml = ''
    with open ("models/Brick/model.sdf", "r") as block3_file:
        block3_xml=block3_file.read().replace('\n', '')

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick3", block3_xml, "/",
                               block3_pose, block3_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_brick4(block4_pose=Pose(position=Point(x=0.65, y=-0.8, z=1.13)),
                       block4_reference_frame="world"):

    print("LOADING BRICK 4")
    # Load Brick SDF
    block4_xml = ''
    with open ("models/Brick/model.sdf", "r") as block4_file:
        block4_xml=block4_file.read().replace('\n', '')

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick4", block4_xml, "/",
                               block4_pose, block4_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_brick5(block5_pose=Pose(position=Point(x=0.65, y=0.8, z=1.13)),
                       block5_reference_frame="world"):

    print("LOADING BRICK 5")
    # Load Brick SDF
    block5_xml = ''
    with open ("models/Brick/model.sdf", "r") as block5_file:
        block5_xml=block5_file.read().replace('\n', '')

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick5", block5_xml, "/",
                               block5_pose, block5_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_brick6(block6_pose=Pose(position=Point(x=0.65, y=-0.8, z=1.13)),
                       block6_reference_frame="world"):

    print("LOADING BRICK 6")
    # Load Brick SDF
    block6_xml = ''
    with open ("models/Brick/model.sdf", "r") as block6_file:
        block6_xml=block6_file.read().replace('\n', '')

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick6", block6_xml, "/",
                               block6_pose, block6_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_brick7(block7_pose=Pose(position=Point(x=0.65, y=0.8, z=1.13)),
                       block7_reference_frame="world"):

    print("LOADING BRICK 7")
    # Load Brick SDF
    block7_xml = ''
    with open ("models/Brick/model.sdf", "r") as block7_file:
        block7_xml=block7_file.read().replace('\n', '')

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick7", block7_xml, "/",
                               block7_pose, block7_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_brick8(block8_pose=Pose(position=Point(x=0.65, y=-0.8, z=1.13)),
                       block8_reference_frame="world"):

    print("LOADING BRICK 8")
    # Load Brick SDF
    block8_xml = ''
    with open ("models/Brick/model.sdf", "r") as block8_file:
        block8_xml=block8_file.read().replace('\n', '')

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick8", block8_xml, "/",
                               block8_pose, block8_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("table")
        resp_delete = delete_model("brick1")
        resp_delete = delete_model("brick2")
        resp_delete = delete_model("brick3")
        resp_delete = delete_model("brick4")
        resp_delete = delete_model("brick5")
        resp_delete = delete_model("brick6")
        resp_delete = delete_model("brick7")
        resp_delete = delete_model("brick8")

    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))