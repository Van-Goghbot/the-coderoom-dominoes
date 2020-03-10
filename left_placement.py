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
import order
import domino
import subprocess
import right_placement
import time
import simulation

def main():

    rospy.init_node("left_arm")
    left_pnp = right_placement.PickAndPlace('left')

    left = [tuple([float(x) for x in y.split(',')]) for y in str(sys.argv[1])[2:-2].split('), (')]

    movement_count = -1
    #print(left)

    for coord in left:
        movement_count +=2
        print("left")
        print coord
        domino.pickandplace('l',left_pnp,coord[1],coord[0],0.24,0,3.14,coord[2]+3.14/2,0.1,movement_count)

if __name__ == '__main__':
    sys.exit(main())

