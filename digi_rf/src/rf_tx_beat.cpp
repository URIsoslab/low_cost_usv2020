
#include "ros/ros.h"
#include "std_msgs/Int32.h"


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  float hz=1;

  ros::init(argc, argv, "rf_tx_beat");

  ros::NodeHandle n;

  ros::NodeHandle nh_priv("~");
  nh_priv.getParam("hz",hz);  //user define the rf_tx frequency

  ros::Publisher beat_pub = n.advertise<std_msgs::Int32>("/rf/beat", 10);

  ros::Rate loop_rate(hz);
  std_msgs::Int32 beat;
  beat.data=0;
  while (ros::ok())
  {
    beat.data=!beat.data;
    beat_pub.publish(beat);
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}
