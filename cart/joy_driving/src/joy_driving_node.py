#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

throttle_coefficient = 20
steering_coefficient = -40

def joystick_callback(joystick_data):
  forward_throttle = -(joystick_data.axes[5] - 1)
  reverse_throttle = -(joystick_data.axes[2] - 1)
  throttle = throttle_coefficient * (forward_throttle - reverse_throttle)
  steering = steering_coefficient * joystick_data.axes[0]
  twist = Twist()
  twist.linear.x = throttle
  twist.angular.z = steering
  driving_pub.publish(twist)

def cart_mover():
  controller_input = rospy.Subscriber("joy", Joy, joystick_callback)
  global driving_pub 
  driving_pub = rospy.Publisher('/cart/cmd_vel', Twist, queue_size=10)
  rospy.init_node('joy_driving')
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  try:
    cart_mover()
  except rospy.ROSInterruptException:
    pass