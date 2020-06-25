#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def twist_callback(twist_data):
  throttle = twist_data.linear.x
  steering = twist_data.angular.z
  right_wheel_pub.publish(throttle - steering)
  left_wheel_pub.publish(throttle + steering)

def cart_mover():
  driving_input = rospy.Subscriber("/cart/cmd_vel", Twist, twist_callback)
  global right_wheel_pub 
  right_wheel_pub = rospy.Publisher('/cart/right_wheel_controller/command', Float64, queue_size=10)
  global left_wheel_pub 
  left_wheel_pub = rospy.Publisher('/cart/left_wheel_controller/command', Float64, queue_size=10)
  rospy.init_node('cart')
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  try:
    cart_mover()
  except rospy.ROSInterruptException:
    pass
