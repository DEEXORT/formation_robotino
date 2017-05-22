#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import robotino_msgs.msg

data = {}
pose_x = []
pose_y = []
orientation_z = []

def PoseOdom(msg):
  data['position_odom_x'] = msg.pose.position.x
  data['position_odom_y'] = msg.pose.position.y
  data['orientation_odom_z'] = msg.pose.orientation.z
  pose_x.append(msg.pose.position.x)
  pose_y.append(msg.pose.position.y)
  orientation_z.append(msg.pose.orientation.z)

def PoseNorthStar(msg):
  data['position_northstar_x'] = msg.pose.position.x
  data['position_northstar_y'] = msg.pose.position.y
  data['orientation_northstar_z'] = msg.pose.orientation.z
  pose_x.append(msg.pose.position.x)
  pose_y.append(msg.pose.position.y)
  orientation_z.append(msg.pose.orientation.z)

def Filter(pose_x, pose_y, orientation_z): 
  result_x = 0
  result_y = 0
  result_z = 0
  for number in pose_x:
    result_x += number
  for number in pose_y:
    result_y += number
  for number in orientation_z:
    result_z += number

  filter_x = result_x/limit
  filter_y = result_y/limit
  filter_z = result_z/limit

if __name__ == '__main__':
  rospy.init_node('tf_filter')
  limit = 10

  for i in range(limit):
    subOdom = rospy.Subscriber('/north_star', robotino_msgs.msg.NorthStarReadings, PoseOdom)
    subNorthStar = rospy.Subscriber('/odom', geometry_msgs.msg.PoseWithCovariance, PoseNorthStar)

  Filter(pose_x, pose_y, orientation_z)
  rospy.spin()

  rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size = 1000)