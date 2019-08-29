#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"

void ultraCallback1(const sensor_msgs::Range &range_msg1)
{
  ROS_INFO("range1:[%f]", range_msg1.range);
}
void ultraCallback2(const sensor_msgs::Range &range_msg2)
{
  ROS_INFO("range2:[%f]", range_msg2.range);
}
void ultraCallback3(const sensor_msgs::Range &range_msg3)
{
  ROS_INFO("range3:[%f]", range_msg3.range);
}
void ultraCallback4(const sensor_msgs::Range &range_msg4)
{
  ROS_INFO("range4:[%f]", range_msg4.range);
}
void ultraCallback5(const sensor_msgs::Range &range_msg5)
{
  ROS_INFO("range5:[%f]", range_msg5.range);
}
void ultraCallback6(const sensor_msgs::Range &range_msg6)
{
  ROS_INFO("range6:[%f]", range_msg6.range);
}
void ultraCallback7(const sensor_msgs::Range &range_msg7)
{
  ROS_INFO("range7:[%f]", range_msg7.range);
}
void ultraCallback8(const sensor_msgs::Range &range_msg8)
{
  ROS_INFO("range8:[%f]", range_msg8.range);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "ultrasonic");
 
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe("/ultrasonic1", 1, ultraCallback1);
  ros::Subscriber sub2 = nh.subscribe("/ultrasonic2", 1, ultraCallback2);
  ros::Subscriber sub3 = nh.subscribe("/ultrasonic3", 1, ultraCallback3);
  ros::Subscriber sub4 = nh.subscribe("/ultrasonic4", 1, ultraCallback4);
  ros::Subscriber sub5 = nh.subscribe("/ultrasonic5", 1, ultraCallback5);
  ros::Subscriber sub6 = nh.subscribe("/ultrasonic6", 1, ultraCallback6);
  ros::Subscriber sub7 = nh.subscribe("/ultrasonic7", 1, ultraCallback7);
  ros::Subscriber sub8 = nh.subscribe("/ultrasonic8", 1, ultraCallback8);

  ros::spin();

  return 0;
}

