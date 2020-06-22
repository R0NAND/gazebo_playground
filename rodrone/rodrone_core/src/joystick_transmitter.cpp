#include "joystick_transmitter.h"

#include <rodrone_core/Setpoints.h>
#include <std_msgs/Float64.h>

Transmitter::Transmitter(){
  setpoints_pub_ = nh_.advertise<rodrone_core::Setpoints>("setpoints", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Transmitter::joyCB, this);

  pos_pos_pub_ = nh_.advertise<std_msgs::Float64>("/rodrone/pos_pos_rotor_joint_controller/command", 1);
  pos_neg_pub_ = nh_.advertise<std_msgs::Float64>("/rodrone/pos_neg_rotor_joint_controller/command", 1);
  neg_neg_pub_ = nh_.advertise<std_msgs::Float64>("/rodrone/neg_neg_rotor_joint_controller/command", 1);
  neg_pos_pub_ = nh_.advertise<std_msgs::Float64>("/rodrone/neg_pos_rotor_joint_controller/command", 1);
}

void Transmitter::joyCB(const sensor_msgs::JoyConstPtr& msg){
  std_msgs::Float64 pos_pos_speed;
  std_msgs::Float64 pos_neg_speed;
  std_msgs::Float64 neg_neg_speed;
  std_msgs::Float64 neg_pos_speed;

  pos_pos_speed.data = (msg->axes[5]-1)*750;
  pos_neg_speed.data = -(msg->axes[5]-1)*750;
  neg_neg_speed.data = (msg->axes[5]-1)*750;
  neg_pos_speed.data = -(msg->axes[5]-1)*750;

  pos_pos_pub_.publish(pos_pos_speed);
  pos_neg_pub_.publish(pos_neg_speed);
  neg_neg_pub_.publish(neg_neg_speed);
  neg_pos_pub_.publish(neg_pos_speed);
}
