#include <ros/ros.h>
#include <digi_rf/rf_driver.h>

int running =0;
void MySigintHandler(int sig)
{
	//shut things down
	running=0;
	return;
}


int main (int argc, char **argv)
{

	ros::init(argc, argv, "rf_node");
	ros::NodeHandle nh;
	RF RF_node(nh);
	running =1;
	signal(SIGINT, MySigintHandler);
	ros::Rate loop_rate(100);  //esc need 50 Hz
	while(ros::ok()&running)
	{
		RF_node.NMEA2ROS();
		ros::spinOnce();
	}
	return 0;
}
