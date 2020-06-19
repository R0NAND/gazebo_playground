#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def cart_mover():
  right_wheel_pub = rospy.Publisher('/cart/right_wheel_controller/command', Float64, queue_size=10)
  left_wheel_pub = rospy.Publisher('/cart/left_wheel_controller/command', Float64, queue_size=10)
  rospy.init_node('cart')
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    right_wheel_pub.publish(80)
    right_wheel_pub.publish(80)
    rate.sleep()

if __name__ == '__main__':
  try:
    cart_mover()
  except rospy.ROSInterruptException:
    pass
