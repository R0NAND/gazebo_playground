#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

post_pos = 0
elbow_1_pos = 0
elbow_2_pos = 0

def joystick_callback(pos_data):
  global post_pos
  global elbow_1_pos
  global elbow_2_pos
  post_pos += pos_data.angular.z
  elbow_1_pos += pos_data.angular.x
  elbow_2_pos += pos_data.angular.y
  post_angle_pub.publish(post_pos)
  elbow_1_angle_pub.publish(elbow_1_pos)
  elbow_2_angle_pub.publish(elbow_2_pos)
  left_finger_pub.publish(pos_data.linear.x * 100)
  right_finger_pub.publish(pos_data.linear.x * -100)


def arm_mover():
  controller_input = rospy.Subscriber("/arm/cmd_pos", Twist, joystick_callback)
  global post_angle_pub
  post_angle_pub = rospy.Publisher('/arm/post_controller/command', Float64, queue_size=10)
  global elbow_1_angle_pub
  elbow_1_angle_pub = rospy.Publisher('/arm/elbow_1_controller/command', Float64, queue_size=10)
  global elbow_2_angle_pub
  elbow_2_angle_pub = rospy.Publisher('/arm/elbow_2_controller/command', Float64, queue_size=10)
  global left_finger_pub
  left_finger_pub = rospy.Publisher('/arm/left_finger_controller/command', Float64, queue_size=10)
  global right_finger_pub
  right_finger_pub = rospy.Publisher('/arm/right_finger_controller/command', Float64, queue_size=10)
  rospy.init_node('arm')
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  try:
    arm_mover()
  except rospy.ROSInterruptException:
    pass
