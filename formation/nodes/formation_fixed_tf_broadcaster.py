#!/usr/bin/env python  
import roslib
roslib.load_manifest('formation')
import rospy
import tf
import math
import nav_msgs.msg
import robotino_msgs.msg
import geometry_msgs.msg

data = {}

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    # br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    def filter(msg):
        data['position_x'] = msg.pose.position.x
        data['position_y'] = msg.pose.position.y
        data['position_z'] = msg.pose.position.z
        data['orientation_x'] = msg.pose.orientation.x
        data['orientation_y'] = msg.pose.orientation.y
        data['orientation_z'] = msg.pose.orientation.z

    def hundle_slave_pose(msg,targetname):
        br = tf.TransformBroadcaster()
        if targetname == 'target_1':
            tra = (-0.5, 0.0, 0)
        elif targetname == 'target_2':
            tra = (-2.0, -2.0 , 0)
        else:
            raise ValueError('WTF THAT TARGET?!')
        br.sendTransform(tra,
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         targetname,
                         "robot_1")

    while not rospy.is_shutdown():
        targetname = rospy.get_param('~target')
        robotname = rospy.get_param('~robot')
        # rospy.Subscriber('/{}/odom'.format(robotname), nav_msgs.msg.Odometry, hundle_slave_pose, targetname)
        # rospy.Subscriber('/%s/north_star' % robotname, robotino_msgs.msg.NorthStarReadings, hundle_slave_pose, targetname)
        rospy.Subscriber('/robot_pose_ekf1/odom_combined', geometry_msgs.msg.PoseWithCovarianceStamped, hundle_slave_pose, targetname)
        # rospy.Subscriber('/%s/north_star' % robotname, robotino_msgs.msg.NorthStarReadings,)

        # br.sendTransform((-2.0, 2.0, 0.0),
        #                  tf.transformations.quaternion_from_euler(0, 0, msg.theta),
        #                  rospy.Time.now(),
        #                  "carrot1",
        #                  "turtle1")
        # br.sendTransform((-2.0, -2.0, 0.0),
        #                  (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(),
        #                  "carrot2",
        #                  "turtle1")
        rate.sleep()