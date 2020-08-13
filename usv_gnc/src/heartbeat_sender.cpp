/**
 * This node need to be run to keep pwm and encoder node alive. As soon as this node is killed, those two would quit autonomously.
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Int8.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "heartbeat_sender");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Int8>("heartbeat", 8);
  ros::Rate loop_rate(5);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::Int8 msg;
    msg.data = 0;
    ROS_INFO("%d", msg.data);
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
