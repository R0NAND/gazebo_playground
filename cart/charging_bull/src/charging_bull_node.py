#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

def camera_frame_callback(camera_frame):
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')
  img_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
  height = img_hsv.height
  width = img_hsv.width
  num_red = 0
  x_total = 0
  for x in width:
    for y in height:
      num_red += 1
      x_total+= 1

  center = x_total/num_red
  twist = Twist()
  if(num_red > 100):
    twist.linear.x = 20
    twist.angular.z = 20 * (center - width / 2) / (width / 2)
  driving_pub.publish(twist)


  

def cart_mover():
  camera_frame = rospy.Subscriber("/cart/front_camera/image_raw", Image, camera_frame_callback)
  global driving_pub 
  driving_pub = rospy.Publisher('/cart/cmd_vel', Twist, queue_size=10)
  rospy.init_node('charging_bull')
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  try:
    cart_mover()
  except rospy.ROSInterruptException:
    pass