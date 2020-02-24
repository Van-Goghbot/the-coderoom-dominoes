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
            if self._verbose:
                print("")
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            print("TEST LINE 70")
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
        #print (joint_angles)
        self._guarded_move_to_joint_position(joint_angles)

def load_gazebo_models(table_pose=Pose(position=Point(x=1.2, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block1_pose=Pose(position=Point(x=0.75, y=0.6, z=0.76)),
                       block1_reference_frame="world",
                       block2_pose=Pose(position=Point(x=0.75, y=0.6, z=0.80)),
                       block2_reference_frame="world",
                       block3_pose=Pose(position=Point(x=0.75, y=0.6, z=0.84)),
                       block3_reference_frame="world",
                       block4_pose=Pose(position=Point(x=0.75, y=-0.6, z=0.76)),
                       block4_reference_frame="world",
                       block5_pose=Pose(position=Point(x=0.75, y=-0.6, z=0.80)),
                       block5_reference_frame="world",
                       block6_pose=Pose(position=Point(x=0.75, y=-0.6, z=0.84)),
                       block6_reference_frame="world"):
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

    block3_xml = ''
    with open ("models/Brick/model.sdf", "r") as block3_file:
        block3_xml=block3_file.read().replace('\n', '')

    block4_xml = ''
    with open ("models/Brick/model.sdf", "r") as block4_file:
        block4_xml=block4_file.read().replace('\n', '')

    block5_xml = ''
    with open ("models/Brick/model.sdf", "r") as block5_file:
        block5_xml=block5_file.read().replace('\n', '')

    block6_xml = ''
    with open ("models/Brick/model.sdf", "r") as block6_file:
        block6_xml=block6_file.read().replace('\n', '')

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
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick1", block1_xml, "/",
                               block1_pose, block1_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick2", block2_xml, "/",
                               block2_pose, block2_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick3", block3_xml, "/",
                               block3_pose, block3_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick4", block4_xml, "/",
                               block4_pose, block4_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick5", block5_xml, "/",
                               block5_pose, block5_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick6", block6_xml, "/",
                               block6_pose, block6_reference_frame)
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
    rospy.init_node("ik_pick_and_place_demo")

    hover_distance = 0.1 # meters

    #Convert Euler angles to quaternion
    #Down facing position
    roll = 0
    pitch = 0*3.14*4/4
    yaw = 0
    quat_down = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    #Midpoint
    roll = 0
    pitch = 3.14*3/4
    yaw = 0
    quat_mid = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    #Front facing position
    roll = 0
    pitch = 0
    yaw = 0
    quat_front = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    #Right facing position

    # Initialise pick and place
    left_pnp = PickAndPlace('left', hover_distance)
    right_pnp = PickAndPlace('right', hover_distance)

    def move(arm,x,y,z,r,p,ya):
        quat = tf.transformations.quaternion_from_euler(r,p,ya)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        print("x = {0}, y = {1}, z = {2}, roll = {3}, pitch = {4}, yaw = {5}".format(x,y,z,r,p,ya))

        if arm == 'l':
            left_pnp._servo_to_pose(pose)
        elif arm == 'r':
            right_pnp._servo_to_pose(pose)
        return x,y,z,r,p,ya

    def slow_move(arm,x,y,z,r,p,ya,steps):
        dx = (coord[0] - x)/steps
        dy = (coord[1] - y)/steps
        dz = (coord[2] - z)/steps
        dr = (coord[3] - r)/steps
        dp = (coord[4] - p)/steps
        dya = (coord[5] - ya)/steps

        for i in range(steps):
            print((x+(steps-(i+1))*dx),(y+(steps-(i+1))*dy),(z+(steps-(i+1))*dz),(r+(steps-(i+1))*dr),(p+(steps-(i+1))*dp),(ya+(steps-(i+1))*dya))
            move(arm,(x+(steps-(i+1))*dx),(y+(steps-(i+1))*dy),(z+(steps-(i+1))*dz),(r+(steps-(i+1))*dr),(p+(steps-(i+1))*dp),(ya+(steps-(i+1))*dya))
        return x,y,z,r,p,ya

    def safe_point():
        left_joint_angles = {'left_s0': -0.08000397926829805,
                     'left_s1': -0.9999781166910306,
                     'left_e0': -1.189968899785275,
                     'left_e1': 1.9400238130755056,
                     'left_w0': 0.6699952259595108,
                     'left_w1': 1.030009435085784,
                     'left_w2': -0.4999997247485215
                     }

        right_joint_angles = {'right_s0': 0.08000397926829805,
                      'right_s1': -0.9999781166910306,
                      'right_e0': 1.189968899785275,
                      'right_e1': 1.9400238130755056,
                      'right_w0': -0.6699952259595108,
                      'right_w1': 1.030009435085784,
                      'right_w2': 0.4999997247485215
                      }
        left_pnp._guarded_move_to_joint_position(left_joint_angles)
        right_pnp._guarded_move_to_joint_position(right_joint_angles)

    def place(x,y,z,r,p,ya):
        if y >= 0:
            arm = 'l'
        elif y <= 0:
            arm = 'r'
        move(arm,x,y,z+0.1,r,p,ya)
        move(arm,x,y,z,r,p,ya)
        if arm == 'l':
            left_pnp.gripper_open()
        if arm == 'r':
            right_pnp.gripper_open()
        move(arm,x,y,z+0.1,r,p,ya)
        safe_point()
        return x,y,z+0.1,r,p,ya

    def pick(arm):
        if arm == 'l':
            #Move to 0.7,0.6,0.3,0,3.14/2,0
            brick_place('l',0.9532,-0.7656,-0.8165,2.338,1.129,-0.9882,0.0689)
            left_pnp.gripper_open()
            coord = move('l',0.8,0.6,0.3,0,3.14/2,0)
            left_pnp.gripper_close()
            brick_place('l',0.9532,-0.7656,-0.8165,2.338,1.129,-0.9882,0.0689)
            safe_point()
        elif arm == 'r':
            #Move to 0.7,-0.6,0.3,0,3.14/2,0
            coord = move('r',0.6,-0.6,0.3,0,3.14/2,0)
            right_pnp.gripper_open()
            coord = move('r',0.8,-0.6,0.3,0,3.14/2,0)
            right_pnp.gripper_close()
            coord = move('r',0.6,-0.6,0.3,0,3.14/2,0)
            safe_point()

    def pickandplace(x,y,z,r,p,ya):
        if y >= 0:
            arm = 'l'
        elif y <= 0:
            arm = 'r'
        pick(arm)
        place(x,y,z,r,p,ya)


    def brick_place(arm,s0,s1,e0,e1,w0,w1,w2):
        joint_angles = {'left_s0': s0,
                     'left_s1': s1,
                     'left_e0': e0,
                     'left_e1': e1,
                     'left_w0': w0,
                     'left_w1': w1,
                     'left_w2': w2
                     }
        if arm == 'l':
            left_pnp._guarded_move_to_joint_position(joint_angles)
        elif arm == 'r':
            right_pnp._guarded_move_to_joint_position(joint_angles)

    def knock_down(x,y,z):
        if y >= 0:
            arm = 'l'
            offset = 0.1
        elif y <= 0:
            arm = 'r'
            offset = -0.1
        coord = move(arm,x,y+offset,z,0,3.14,0)
        coord = move(arm,x,y-offset,z,0,3.14,0)

    #Load models
    load_gazebo_models()

    #safe_point()

    #Set up six bricks in a straight line
    pickandplace(0.75,-0.3,0.15,0,3.14,3.14/2)
    pickandplace(0.75,-0.2,0.15,0,3.14,3.14/2)
    pickandplace(0.75,-0.1,0.15,0,3.14,3.14/2)
    pickandplace(0.75,-0.0,0.15,0,3.14,3.14/2)
    pickandplace(0.75,0.1,0.15,0,3.14,3.14/2)
    pickandplace(0.75,0.2,0.15,0,3.14,3.14/2)
    knock_down(0.75,-0.3,0.2)


    #Attempt at Amy's coordinates
    """
    position1 = [0.7692841534814815,-0.5479860820740741,0.15,-1.4832296975868564,3.14,0]
    position2 = [0.9018232865185186,-0.5120001445925926,0.15,-1.053420499535529,3.14,0]
    position3 = [0.9845012361481481,-0.38774743940740747,0.15,-0.14841348102091267,3.14,0]
    position4 = [0.973423472,-0.23583320800000002,0.15,0.21738343878129116,3.14,0]
    position5 = [0.9322231851851851,-0.09240292592592596,0.15,0.3201278688721856,3.14,0]
    position6 = [0.8848077439999998,0.04609798400000002,0.15,0.3266637223869834,3.14,0]
    position7 = [0.8407259211851852,0.191439770074074,0.15,0.24265457744601776,3.14,0]
    position8 = [0.821643264,0.340000704,0.15,-0.04455551744798016,3.14,0]
    position9 = [0.8777552574814814,0.47205806192592603,0.15,-0.8439595442470305,3.14,0]
    position10 = [1.010180084148148,0.5266014885925927,0.15,-1.4168222996663273,3.14,0]
    positions = [position1,position2,position3,position4,position5,position6,position7,position8,position9,position10]

    for i in range(len(positions)):
        pose = positions[i]
        if pose[1]>=0:
            arm = 'l'
        if pose[1]<0:
            arm = 'r'
        safe_point()
        move(arm,-0.1+pose[0],pose[1],pose[2],pose[5],pose[4],pose[3])
        safe_point()
    """
    safe_point()
    delete_gazebo_models()


if __name__ == '__main__':
    sys.exit(main())