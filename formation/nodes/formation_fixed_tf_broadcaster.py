#!/usr/bin/env python  
import roslib
roslib.load_manifest('formation')
import rospy
import tf
import math
import nav_msgs.msg
import robotino_msgs.msg

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    # br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
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
        # rospy.Subscriber('/{}/pose'.format(robotname), nav_msgs.msg.Odometry, hundle_slave_pose, targetname)
        rospy.Subscriber('/%s/north_star' % robotname, robotino_msgs.msg.NorthStarReadings, hundle_slave_pose, targetname)

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