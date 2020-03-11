
#!/usr/bin/env python
#/robot/range/left_hand_range
import rospy
import time
from sensor_msgs.msg import Range

class Range_Listener():
    def callback(self, msg):
        self.range = msg.range

    def __init__(self):
        rospy.Subscriber("/robot/range/left_hand_range/state",Range,self.callback,queue_size = 1)
        self.rate = rospy.Rate(10.0)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('infra_red_listener')
    rospy.Rate(10.0)
    listener = Range_Listener()
    print(listener.range)