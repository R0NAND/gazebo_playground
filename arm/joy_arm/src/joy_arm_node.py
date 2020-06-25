#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def joystick_callback(joystick_data):
  post_pos = joystick_data.axes[0] * 0.1
  elbow_1_pos = joystick_data.axes[1] * 0.03
  elbow_2_pos = joystick_data.axes[4] * 0.03
  twist = Twist()
  twist.angular.x = elbow_1_pos
  twist.angular.y = elbow_2_pos
  twist.angular.z = post_pos
  twist.linear.x = joystick_data.buttons[5]
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