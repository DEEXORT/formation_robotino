#!/usr/bin/env python  
import roslib
roslib.load_manifest('formation')
import rospy
import math
import tf
import geometry_msgs.msg
import robotino_msgs.msg
import omniturtle.srv
from nav_msgs.msg import Odometry

data = {}

def position(msg):
    data['position_x'] = msg.pose.position.x
    data['position_y'] = msg.pose.position.y
    data['orientation_z'] = msg.pose.orientation.z 

def callback(msg):
    global pose
    pose = msg

if __name__ == '__main__':
    rospy.init_node('tf_robotino')

    listener = tf.TransformListener()

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', omniturtle.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    robotino_vel = rospy.Publisher('robot_2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1000)

    pose = Odometry()
    p = 5.0   

    rate = rospy.Rate(10.0)
    # listener.waitForTransform("/turtle2", "/carrot1", rospy.Time(), rospy.Duration(4.0))
    # sub = rospy.Subscriber('robot_1/north_star', robotino_msgs.msg.NorthStarReadings, position)
    sub = rospy.Subscriber('robot_1/robot_pose_ekf/odom_combined', geometry_msgs.msg.PoseWithCovarianceStamped, position)


    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform("/robot_2", "/target_1", rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            continue

        cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = (pose.x - trans[0]) * p
        # cmd.linear.y = (pose.y - trans[1]) * p
        cmd.linear.x = p * trans[0]
        cmd.linear.y = p * trans[1]
        # cmd.angular.z = b * math.atan2(trans[1], trans[0])

        euler = tf.transformations.euler_from_quaternion(rot)           
        cmd.angular.z = euler[2] * p
        #Tu = 1
        #Ku = 8
        #Kp = 0.6 * Ku
        #Ki = 2 * Kp / Tu
        #Kd = Kp * Tu / 8
        #angular = 8 * math.atan2(trans[1], trans[0])
        #linear = 7.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        #cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        robotino_vel.publish(cmd)

        rate.sleep()