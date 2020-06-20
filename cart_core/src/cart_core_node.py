#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

def joystick_callback(joystick_data):
  throttle = -(joystick_data.axes[5] - 1)*10
  steering = -joystick_data.axes[0]*30
  right_wheel_pub.publish(throttle - steering)
  left_wheel_pub.publish(throttle + steering)

def cart_mover():
  controller_input = rospy.Subscriber("joy", Joy, joystick_callback)
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
