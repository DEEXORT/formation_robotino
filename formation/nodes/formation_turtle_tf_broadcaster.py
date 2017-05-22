#!/usr/bin/env python  
import roslib
roslib.load_manifest('formation')
import rospy
import tf
import robotino_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg

def handle_robot_pose(msg, robotname):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.pose.pose.orientation.z), rospy.Time.now(), robotname, "world")

if __name__ == '__main__':
    rospy.init_node('formation_turtle_tf_broadcaster')
    robotname = rospy.get_param('~robot')
    number = rospy.get_param('~number')
    # rospy.Subscriber('/%s/pose' % robotname, nav_msgs.msg.Odometry, handle_robot_pose, robotname)
    # rospy.Subscriber('/%s/north_star' % robotname, robotino_msgs.msg.NorthStarReadings, handle_robot_pose, robotname)
    # rospy.Subscriber('/{}/robot_pose_ekf/odom_combined'.format(robotname), geometry_msgs.msg.PoseWithCovarianceStamped, handle_robot_pose, robotname
    rospy.Subscriber('/robot_pose_ekf{}/odom_combined'.format(number), geometry_msgs.msg.PoseWithCovarianceStamped, handle_robot_pose, robotname)
    rospy.spin()