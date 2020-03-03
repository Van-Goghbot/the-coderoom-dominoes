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

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.10, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        rospy.sleep(1.0)

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()

    def gripper_close(self):
        self._gripper.close()

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        #print (joint_angles)
        self._guarded_move_to_joint_position(joint_angles)

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

def load_brick(r_count):
    if r_count == 0:
        load_brick1()
    elif count == 1:
        load_rbrick2
    elif count == 2:
        load_rbrick2
    elif count == 3:
        load_rbrick2
    elif count == 4:
        load_rbrick2
    elif count == 5:
        load_rbrick2
    elif count == 6:
        load_rbrick2


def load_brick1(block1_pose=Pose(position=Point(x=0.65, y=-0.8, z=1.13)),
                       block1_reference_frame="world"):
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

def load_brick2(block2_pose=Pose(position=Point(x=0.65, y=0.8, z=1.13)),
                       block2_reference_frame="world"):
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

    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    """Simple pick and place example"""
    rospy.init_node("conservative")
    hover_distance = 0.1 # meters

    # Initialise pick and place
    right_pnp = PickAndPlace('right', hover_distance)
    left_test = PickAndPlace('left', hover_distance)

    domino.safe_point_r(right_pnp)
    domino.safe_point_l(left_test)

    #Get coordinates from the user
    translations, angles = listener.get_coordinates()
    start_x = translations[0][0] * 100 + 60
    start_y = (translations[0][1] + 0.1) * -100 + 120
    start_angle = angles[0][2] - 3.1415/2
    end_x = translations[1][0] * 100 + 60
    end_y = (translations[1][1] + 0.1) * -100 + 120
    end_angle = angles[1][2] + 3.1415/2
    print start_x, start_y, ((3.14/2) - start_angle)
    print end_x, end_y, ((3.14/2) + end_angle)

    #Get Bezier coordinates
    coords = bezier_interpolation.create_path(start_x, start_y, start_angle, end_x, end_y, end_angle)
    #coords = bezier_interpolation.create_path(10,35,20,110,40,10)

    def run_pnp(coords):

        check_list = []
        right = []
        left = []

        for brick in coords:
            #print("brick")
            #print float(brick.y), float(brick.x), float(brick.rot)
            if brick.x <= 0:
                right.append((float(brick.x), float(brick.y), float(brick.rot)))
            else:
                left.append((float(brick.x), float(brick.y), float(brick.rot)))
            error_check = domino.ik_test(round(brick.y, 3),round(brick.x, 3),0.23,0,3.14,brick.rot,right_pnp,left_test)
            if error_check[0] == False:
                check_list.append(error_check[0])
            if error_check[1] == False:
                check_list.append(error_check[1])

        right.sort()

        right = right[::-1]
        #print('right')
        #print(right)

        left.sort()
        #print('left')
        #print(left)

        ordered_coords = left + right

        #print('coord\n' + str(coords))
        #print('ordered\n' + str(ordered_coords))

        #Load models
        #load_table()

        if len(check_list) > 0:
            print ("Failed Path")
        else:
            print("Succesful Path")
            rc = subprocess.Popen("python left_placement.py '" + str(left) + "'", shell=True)
            for coord in right:
                print("right")
                print coord
                domino.pickandplace('r',right_pnp,coord[1],coord[0],0.23,0,3.14,coord[2],0.15)
        return check_list

    check_list = run_pnp(coords)
    run_number = 0
    while len(check_list) > 0:
    	run_number += 1
    	print("")
    	print("Run number {0} failed".format(str(run_number)))

        start_x = start_x
        start_y = start_y
        start_angle = start_angle -0.2
        end_x = end_x
        end_y = end_y
        end_angle = end_angle +0.2

    	print(start_x, start_y, start_angle, end_x, end_y, end_angle)
    	coords = bezier_interpolation.create_path(10,35,0,110,40,0)
    	check_list = run_pnp(coords)
    	if run_number >= 1:
    		break


if __name__ == '__main__':
    sys.exit(main())
