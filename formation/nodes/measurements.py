#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('formation')
import robotino_msgs.msg
import math
import tf

file = open('position_x.txt', 'w')
file_2 = open('number.txt', 'w')
data = {}

def Pose(msg):
	data['position_x'] = msg.pose.position.x
	pose = str(msg.pose.position.x)
	file.write(pose + '\n')
	file_2.write('1' + '\n')
	# file.close()

if __name__ == '__main__':
	rospy.init_node('tf_filter')

	sub = rospy.Subscriber('/robot_1/north_star', robotino_msgs.msg.NorthStarReadings, Pose)
	rospy.spin()