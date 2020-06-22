#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

def joystick_callback(joystick_data):
  angle = -joystick_data.axes[0]
  post_angle_pub.publish(angle)

def cart_mover():
  controller_input = rospy.Subscriber("joy", Joy, joystick_callback)
  global post_angle_pub
  post_angle_pub = rospy.Publisher('/arm/elbow_2_controller/command', Float64, queue_size=10)
  rospy.init_node('arm')
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  try:
    cart_mover()
  except rospy.ROSInterruptException:
    pass
