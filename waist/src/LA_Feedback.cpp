#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"

//float Feedback[3];

void FeedbackCallback(const geometry_msgs::Vector3 &Feedback_msg)
{
  //Feedback[0] = Feedback_msg.x;
  //Feedback[1] = Feedback_msg.y;
  //Feedback[2] = Feedback_msg.z;

  ROS_INFO("LA1:[%f], LA2:[%f], LA3[%f]", Feedback_msg.x, Feedback_msg.y, Feedback_msg.z);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "LA_Feedback");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/LA_Feedback", 1, FeedbackCallback);

  ros::spin();

  return 0;
}

