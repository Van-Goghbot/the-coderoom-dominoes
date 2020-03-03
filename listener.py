#!/usr/bin/env python
import rospy
import tf

def get_coordinates():
	#rospy.init_node('example_tf_listener')
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)

	translations = []
	angles = []

	quantity = 0
	while not rospy.is_shutdown() and quantity != 2:
		try:
			(l_trans,l_rot) = listener.lookupTransform('left_gripper', 'base', rospy.Time(0))
			print l_trans,l_rot
			(r_trans,r_rot) = listener.lookupTransform('right_gripper', 'base', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		ready = raw_input('Are you ready to record? [Y/N]: ').upper()
		
		if ready == 'Y':
			arm = raw_input('Which arm are you looking for? [L/R]: ').upper()
			if arm == "L":
				trans, rot = l_trans, l_rot
			else:
				trans, rot = r_trans, r_rot
			Translation = trans
			Quaternion = rot
			Angle = tf.transformations.euler_from_quaternion(rot)
			translations.append(trans)
			angles.append(tf.transformations.euler_from_quaternion(rot))
			print("Translation: ", Translation)
			print("Quaternion: ", Quaternion)
			print("Angles: ", Angle)
			print("")
			quantity += 1
		rate.sleep()
		print translations,angles
	return translations, angles

if __name__ == '__main__':
	rospy.init_node("ik_pick_and_place_demo")
	get_coordinates()