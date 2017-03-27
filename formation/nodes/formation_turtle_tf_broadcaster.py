#!/usr/bin/env python  
import roslib
roslib.load_manifest('formation')
import rospy
import tf
import robotino_msgs.msg
import nav_msgs.msg

def handle_robot_pose(msg, robotname):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.pose.orientation.z), rospy.Time.now(), robotname, "world")

if __name__ == '__main__':
    rospy.init_node('formation_turtle_tf_broadcaster')
    robotname = rospy.get_param('~robot')
    # rospy.Subscriber('/%s/pose' % robotname, nav_msgs.msg.Odometry, handle_robot_pose, robotname)
    rospy.Subscriber('/%s/north_star' % robotname, robotino_msgs.msg.NorthStarReadings, handle_robot_pose, robotname)
    rospy.spin()