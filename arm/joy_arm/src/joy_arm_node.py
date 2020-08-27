#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

base_angle_coefficient = 0.03 # Radians per second per joystick input
elbow_angle_coefficient = 0.03 # Radians per second per joystick input


def joystick_callback(joystick_data):
  post_rate = joystick_data.axes[0] * base_angle_coefficient
  elbow_1_rate = joystick_data.axes[1] * elbow_angle_coefficient
  elbow_2_rate = joystick_data.axes[4] * elbow_angle_coefficient
  twist = Twist()  #I am conveniently using the twist message to store the three motor angles, and the gripper value in x
  twist.angular.x = elbow_1_rate
  twist.angular.y = elbow_2_rate
  twist.angular.z = post_rate
  twist.linear.x = joystick_data.buttons[5] * 100
  arm_pub.publish(twist)

def cart_mover():
  controller_input = rospy.Subscriber("joy", Joy, joystick_callback)
  global arm_pub 
  arm_pub = rospy.Publisher('/arm/cmd_pos', Twist, queue_size=10)
  rospy.init_node('joy_arm')
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  try:
    cart_mover()
  except rospy.ROSInterruptException:
    pass