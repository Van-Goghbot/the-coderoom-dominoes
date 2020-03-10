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
import simulation
import math

class PickAndPlace(object):
    def __init__(self, limb, verbose=True):
        self._limb_name = limb # string
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

def main():
    """Simple pick and place example"""
    rospy.init_node("right_arm")
    # domino.setup_listener_left()

    #Change depending on whether running simulation or not
    mode_sim = True

    # Initialise pick and place
    right_pnp = PickAndPlace('right')
    left_test = PickAndPlace('left')

    domino.safe_point_r(right_pnp)
    domino.safe_point_l(left_test)

    #Get coordinates from the user
    translations, angles = listener.get_coordinates()
    start_x = translations[0][0] * 100 + 60
    start_y = (translations[0][1] + 0.1) * -100 + 120
    start_angle = angles[0][2] - math.pi/2
    end_x = translations[1][0] * 100 + 60
    end_y = (translations[1][1] + 0.1) * -100 + 120
    end_angle = angles[1][2] + math.pi/2
    print start_x, start_y, ((math.pi) - start_angle)
    print end_x, end_y, ((math.pi) + end_angle)

    #Get Bezier coordinates for a path
    if mode_sim == True:
        coords = bezier_interpolation.create_path(10, 35, -3.14/8, 110, 40, 3.14/8, 110)
    else:
        coords = bezier_interpolation.create_path(start_x, start_y, start_angle, end_x, end_y, end_angle, 110)
    #coords = bezier_interpolation.create_path(10,35,20,110,40,10)

    def run_pnp(coords):

        #Initialise necessary lists
        check_list = []
        right = []
        left = []

        #Setup variables
        table_height = 0.24
        hover = 0.1

        #Setup variables for height adjustment
        table_length = 160 #160cm
        table_reach = table_length/2 #Each arm only uses half the table
        relaive_table_length = 0.6 #Maximum y-distance travelled in gazebo by arm
        scale_factor = table_reach/relaive_table_length #Scaling factor
        scale_difference = 317.4 #Conversion between gazebo and millimetres
        #Table Heights
        raised_height = 79 #End with bricks under
        lowered_height = 72
        height_difference = raised_height - lowered_height
        incline_angle = float(math.tan(height_difference/table_length))
        print ("Incline Angle")
        print (float(incline_angle))

        #For each brick check inverse kinematics of placement
        for brick in coords:
            print("brick")
            print float(brick.y), float(brick.x), float(brick.rot)
            if brick.x <= 0:
                right.append((float(brick.x), float(brick.y), float(brick.rot)))
            else:
                left.append((float(brick.x), float(brick.y), float(brick.rot)))

            adjusted_z = table_height + ((float(brick.y)+relaive_table_length)*scale_factor*incline_angle)/scale_difference
            error_check = domino.ik_test(round(brick.y, 3),round(brick.x, 3),adjusted_z,incline_angle,math.pi,brick.rot,hover,right_pnp,left_test)
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

        if len(check_list) > 0:
            print ("Failed Path")
        else:
            print("Succesful Path")

            #Load table model
            simulation.load_table()

            #Call function to run the left arm simultaneously
            rc = subprocess.Popen("python left_placement.py '" + str(left) + "'", shell=True)
            #Pause to avoid coliision with the left arm
            rospy.sleep(10)

            #Count how many moves have been made (To load bricks in simulation)
            movement_count = 0

            for coord in right:
                movement_count += 2

                adjusted_z = table_height + ((float(brick.y)+0.6)*scale_factor*incline_angle)/scale_difference

                print("right")
                print coord
                domino.pickandplace('r',right_pnp,coord[1],coord[0],adjusted_z,incline_angle,math.pi,coord[2]+math.pi/2,hover,movement_count)
        return check_list

    check_list = run_pnp(coords)

    #How many paths have we checked?
    run_number = 0

    influence = 110

    #If the first run failed, rerun it
    while len(check_list) > 0:
        influence -= 10
        run_number += 1
        print("")
        print("Run number {0} failed".format(str(run_number)))
        #Rerun path generation with smoother path
        #coords = bezier_interpolation.create_path(10,35,0,110,40,0)
        if mode_sim == True:
            coords = bezier_interpolation.create_path(10, 35, -3.14/8, 110, 40, 3.14/8, influence)
        else:
            coords = bezier_interpolation.create_path(start_x, start_y, start_angle, end_x, end_y, end_angle, influence)

        #Rerun Ik check for new path
        check_list = run_pnp(coords)

        #Break after x number of iterations
        if run_number >= 10:
            break

    simulation.delete_gazebo_models()

if __name__ == '__main__':
    sys.exit(main())
