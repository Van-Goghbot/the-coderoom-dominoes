"""
Simple Pick And Place Demo
Modified from Baxter SDK example (https://sdk.rethinkrobotics.com/wiki/Home)

Last modified by : R Saputra
2020
"""

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
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
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

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
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
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
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        print('Approaching')
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        print('Ready to grip')
        # close gripper
        self.gripper_close()
        print('grip')
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def load_gazebo_models(table_pose=Pose(position=Point(x=0.95, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block1_pose=Pose(position=Point(x=0.5625, y=0.1595, z=0.7825)),
                       block1_reference_frame="world",
                       block2_pose=Pose(position=Point(x=0.9, y=0.1595, z=0.7825)),
                       block2_reference_frame="world"):
    # Load Table SDF
    table_xml = ''
    with open ("models/table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    # Load Brick SDF
    block1_xml = ''
    with open ("models/Brick/model.sdf", "r") as block1_file:
        block1_xml=block1_file.read().replace('\n', '')

    block2_xml = ''
    with open ("models/Brick/model.sdf", "r") as block2_file:
        block2_xml=block2_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("brick1", block1_xml, "/",
                               block1_pose, block1_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("brick2", block2_xml, "/",
                               block2_pose, block2_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("table")
        resp_delete = delete_model("brick1")
        resp_delete = delete_model("brick2")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    """Simple pick and place example"""
    rospy.init_node("ik_pick_and_place_demo")

    hover_distance = 0.1 # meters

    #Convert Euler angles to quaternion
    #Down facing position
    roll = 0
    pitch = 3.14*4/4
    yaw = 0
    quat_down = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    #Front facing position
    roll = 0
    pitch = 3.14*4/4
    yaw = 0
    quat_front = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

    # Start pose for left arm
    left_pose = Pose()
    left_pose.position.x = 0.579679836383
    left_pose.position.y = 0.283311769707
    left_pose.position.z = 0.213676720426
    left_pose.orientation.x = -0.0249590815779
    left_pose.orientation.y = 0.999649402929
    left_pose.orientation.z = 0.00737916180073
    left_pose.orientation.w = 0.00486450832011

    # Starting Pose for right arm
    right_pose = Pose()
    right_pose.position.x = 0.579679836383
    right_pose.position.y = -0.283311769707
    right_pose.position.z = 0.213676720426
    right_pose.orientation.x = -0.0249590815779
    right_pose.orientation.y = 0.999649402929
    right_pose.orientation.z = -0.00737916180073
    right_pose.orientation.w = 0.00486450832011

    # Raised picking pose
    pick_up = Pose()
    pick_up.position.x = 0.6
    pick_up.position.y = 0.45
    pick_up.position.z = 0.2
    pick_up.orientation.x = quat_down[0]
    pick_up.orientation.y = quat_down[1]
    pick_up.orientation.z = quat_down[2]
    pick_up.orientation.w = quat_down[3]

    # Lowered picking pose
    pick_down = Pose()
    pick_down.position.x = 0.6
    pick_down.position.y = 0.45
    pick_down.position.z = 0.1 #Change once brick dimensions are confirmed
    pick_down.orientation.x = quat_down[0]
    pick_down.orientation.y = quat_down[1]
    pick_down.orientation.z = quat_down[2]
    pick_down.orientation.w = quat_down[3]

    # Raised placing pose
    place_up = Pose()
    place_up.position.x = 0.8
    place_up.position.y = 0.3
    place_up.position.z = 0.2
    place_up.orientation.x = quat_front[0]
    place_up.orientation.y = quat_front[1]
    place_up.orientation.z = quat_front[2]
    place_up.orientation.w = quat_front[3]

    # Brick 2 Raised picking pose
    pick_up2 = Pose()
    pick_up2.position.x = 0.9
    pick_up2.position.y = 0.45
    pick_up2.position.z = 0.2
    pick_up2.orientation.x = quat_down[0]
    pick_up2.orientation.y = quat_down[1]
    pick_up2.orientation.z = quat_down[2]
    pick_up2.orientation.w = quat_down[3]

    # Brick 2 Lowered picking pose
    pick_down2 = Pose()
    pick_down2.position.x = 0.9
    pick_down2.position.y = 0.45
    pick_down2.position.z = 0.1 #Change once brick dimensions are confirmed
    pick_down2.orientation.x = quat_down[0]
    pick_down2.orientation.y = quat_down[1]
    pick_down2.orientation.z = quat_down[2]
    pick_down2.orientation.w = quat_down[3]

    # Initialise pick and place
    left_pnp = PickAndPlace('left', hover_distance)
    right_pnp = PickAndPlace('right', hover_distance)

    # Go to start arm position
    left_pnp.move_to_start(left_pnp.ik_request(left_pose))
    #right_pnp.move_to_start(right_pnp.ik_request(right_pose))

    #Load models
    load_gazebo_models()

    #Pick up brick
    left_pnp.move_to_start(left_pnp.ik_request(pick_up))
    left_pnp.gripper_open()
    left_pnp.move_to_start(left_pnp.ik_request(pick_down))
    left_pnp.gripper_close()
    left_pnp.move_to_start(left_pnp.ik_request(pick_up))

    #Place brick
    left_pnp.move_to_start(left_pnp.ik_request(place_up))
    left_pnp.gripper_close()

    #Pick up brick 2
    left_pnp.move_to_start(left_pnp.ik_request(pick_up2))
    left_pnp.gripper_open()
    left_pnp.move_to_start(left_pnp.ik_request(pick_down2))
    left_pnp.gripper_close()
    left_pnp.move_to_start(left_pnp.ik_request(pick_up2))   


    delete_gazebo_models()


if __name__ == '__main__':
    sys.exit(main())

