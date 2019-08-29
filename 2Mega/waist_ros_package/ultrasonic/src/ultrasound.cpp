#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt16.h"

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

/*void ultraCallback1(const std_msgs::UInt16 &range_msg1)
{
  ROS_INFO("range1:[%d]", range_msg1.data);
}
void ultraCallback2(const std_msgs::UInt16 &range_msg2)
{
  ROS_INFO("range2:[%d]", range_msg2.data);
}
void ultraCallback3(const std_msgs::UInt16 &range_msg3)
{
  ROS_INFO("range3:[%d]", range_msg3.data);
}
void ultraCallback4(const std_msgs::UInt16 &range_msg4)
{
  ROS_INFO("range4:[%d]", range_msg4.data);
}
void ultraCallback5(const std_msgs::UInt16 &range_msg5)
{
  ROS_INFO("range5:[%d]", range_msg5.data);
}
void ultraCallback6(const std_msgs::UInt16 &range_msg6)
{
  ROS_INFO("range6:[%d]", range_msg6.data);
}
void ultraCallback7(const std_msgs::UInt16 &range_msg7)
{
  ROS_INFO("range7:[%d]", range_msg7.data);
}
void ultraCallback8(const std_msgs::UInt16 &range_msg8)
{
  ROS_INFO("range8:[%d]", range_msg8.data);
}
*/

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "ultrasound");
 
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe("/ultrasound1", 1, ultraCallback1);
  ros::Subscriber sub2 = nh.subscribe("/ultrasound2", 1, ultraCallback2);
  ros::Subscriber sub3 = nh.subscribe("/ultrasound3", 1, ultraCallback3);
  ros::Subscriber sub4 = nh.subscribe("/ultrasound4", 1, ultraCallback4);
  ros::Subscriber sub5 = nh.subscribe("/ultrasound5", 1, ultraCallback5);
  ros::Subscriber sub6 = nh.subscribe("/ultrasound6", 1, ultraCallback6);
  ros::Subscriber sub7 = nh.subscribe("/ultrasound7", 1, ultraCallback7);
  ros::Subscriber sub8 = nh.subscribe("/ultrasound8", 1, ultraCallback8);

  ros::spin();

  return 0;
}

