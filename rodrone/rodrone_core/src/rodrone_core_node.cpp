#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "joystick_transmitter.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rodrone");
  Transmitter transmitter;
  ros::spin();
}