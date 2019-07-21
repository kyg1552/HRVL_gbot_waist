#include "ros/ros.h"
#include "std_msgs/Float64.h"

void tCallback(const std_msgs::Float64 &t)
{

  ROS_INFO("time: [%f]", t.data);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "time");
 
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/time", 10, tCallback);

  ros::spin();

  return 0;
}

