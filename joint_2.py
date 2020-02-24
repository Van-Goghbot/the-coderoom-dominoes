"""
Simple Joint Position Control Example
Modified from Baxter SDK example (https://sdk.rethinkrobotics.com/wiki/Home)
 
Last modified by : R Saputra
2020
"""

import rospy
import baxter_interface


class SimpleJointPositionControl(object):

    #Initialise Joint position control via vaxter_interface.Limb
    def __init__(self, limb, speed=0.3, accuracy=baxter_interface.settings.JOINT_ANGLE_TOLERANCE):
        # Create baxter_interface limb instance
        self._arm = limb
        while True:
          try:
              self._limb = baxter_interface.Limb(self._arm)
          except:
              continue
          break
        
        # Parameters which will describe joint position moves
        self._speed = speed
        self._accuracy = accuracy

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        if not self._init_state:
          print("Enabling robot... ")
          self._rs.enable()

    def move_to(self, joint_angles):
        """
        Send target joint position to the joint position controller
        """
        rospy.sleep(1.0)

        rospy.loginfo("Robot move to joint position started...")
        print("  Press Ctrl-C to stop...")

        # Set joint position speed ratio for execution
        self._limb.set_joint_position_speed(self._speed)

        # Send joint angles command
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles, timeout=20.0, threshold=self._accuracy)
        else:
            rospy.logerr("No Joint Angles provided...")

        # Set joint position speed back to default
        self._limb.set_joint_position_speed(0.3)

        
def main():
    print("Initializing node... ")
    rospy.init_node("simple_joint_controller_example")

    left_arm = SimpleJointPositionControl('left')
    right_arm = SimpleJointPositionControl('right')

    # Start sending joint position command
    print('[Start Demo]')
    left_joint_angles = {'left_e0': -0.14304370847031483,
                         'left_e1': 1.476840003536682,
                         'left_s0': -0.20862138715241627,
                         'left_s1': -0.5733253194721734,
                         'left_w0': 0.8114758367913839,
                         'left_w1': -0.6979612584878633,
                         'left_w2': -0.5296068670174391
                         }
    print('Move the left arm')
    left_arm.move_to(left_joint_angles)
    
    right_joint_angles = {'right_e0': 0.14304370847031483,
                          'right_e1': 1.476840003536682,
                          'right_s0': 0.20862138715241627,
                          'right_s1': -0.5733253194721734,
                          'right_w0': -0.8114758367913839,
                          'right_w1': -0.6979612584878633,
                          'right_w2': 0.5296068670174391
                          }
    print('Move the right arm')
    right_arm.move_to(right_joint_angles)
        
    left_joint_angles = {'left_s0': 0,
                         'left_s1': 0,
                         'left_e0': 0,
                         'left_e1': 0,
                         'left_w0': 0,
                         'left_w1': 0,
                         'left_w2': 0
                         }
    #print('Move the left arm')
    #left_arm.move_to(left_joint_angles)

    right_joint_angles = {'right_s0': 0,
                          'right_s1': 0,
                          'right_e0': 0,
                          'right_e1': 0,
                          'right_w0': 0,
                          'right_w1': 0,
                          'right_w2': 0
                          }
    #print('Move the right arm')
    #right_arm.move_to(right_joint_angles)
    print('[End of Demo]')

if __name__ == '__main__':
    main()
