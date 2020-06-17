#ifndef RODRONE_CORE_JOYSTICK_TRANSMITTER
#define RODRONE_CORE_JOYSTICK_TRANSMITTER

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

class Transmitter
{
  public:
    Transmitter();

  private:
    ros::NodeHandle nh_;
    ros::Publisher setpoints_pub_;
    ros::Subscriber joy_sub_;

    ros::Publisher pos_pos_pub_;
    ros::Publisher pos_neg_pub_;
    ros::Publisher neg_neg_pub_;
    ros::Publisher neg_pos_pub_;

    void joyCB(const sensor_msgs::JoyConstPtr& msg);
};

#endif //RODRONE_CORE_JOYSTICK_TRANSMITTER
