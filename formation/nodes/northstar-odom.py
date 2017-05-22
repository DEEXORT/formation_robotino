#!/usr/bin/env python
import geometry_msgs.msg
import nav_msgs.msg
import robotino_msgs.msg
import rospy

def position(msg):
	navmsg = nav_msgs.msg.Odometry()
	navmsg.pose.pose = msg.pose
	navmsg.header.seq = msg.seq
	navmsg.header.stamp = msg.stamp
	navmsg.header.frame_id = 'robot1/northstar'
	navmsg.child_frame_id = 'base_link'

	pub = rospy.Publisher('/{}/northstar_odom'.format(robotname), nav_msgs.msg.Odometry, queue_size=10000)
	# rospy.loginfo("Republishing {}...".format(navmsg))
	pub.publish(navmsg)

if __name__ == '__main__':
	rospy.init_node('north_odom_converter')

	robotname = rospy.get_param('~robot')
	sub = rospy.Subscriber('/{}/north_star'.format(robotname), robotino_msgs.msg.NorthStarReadings, position)
	# sub = rospy.Subscriber('/north_star', robotino_msgs.msg.NorthStarReadings, position)
	rospy.spin()