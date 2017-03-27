#!/usr/bin/env python  
import roslib
roslib.load_manifest('formation')
import rospy
import math
import tf
import geometry_msgs.msg
import omniturtle.srv
from omniturtle.msg import Pose

def callback(msg):
    global pose
    pose = msg

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', omniturtle.srv.Spawn)
    spawner(2, 2, 0, 'turtle3')

    turtle_vel = rospy.Publisher('turtle3/cmd_vel', geometry_msgs.msg.Twist, queue_size=1000)


    pose = Pose()
    p = 5.0  
    b = 6.0 

    rate = rospy.Rate(10.0)
    # listener.waitForTransform("/turtle3", "/carrot2", rospy.Time(), rospy.Duration(4.0))
    sub = rospy.Subscriber('turtle1/Pose', Pose, callback, queue_size = 1000)


    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform("/turtle3", "/carrot2", rospy.Time(0))
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
        turtle_vel.publish(cmd)

        rate.sleep()