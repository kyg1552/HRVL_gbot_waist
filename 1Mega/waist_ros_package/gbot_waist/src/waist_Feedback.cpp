#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"

void FeedbackCallback(const geometry_msgs::Vector3 &Feedback_msg)
{
  ROS_INFO("LA1:[%f], LA2:[%f], LA3[%f]", Feedback_msg.x, Feedback_msg.y, Feedback_msg.z);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "waist_Feedback");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/waist_Feedback", 1, FeedbackCallback);

  ros::spin();

  return 0;
}

