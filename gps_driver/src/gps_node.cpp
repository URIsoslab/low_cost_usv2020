#include <ros/ros.h>
#include <gps_driver.h>

int running =0;
void MySigintHandler(int sig)
{
	//shut things down
	running=0;
	return;
}


int main (int argc, char **argv)
{

	ros::init(argc, argv, "gps_node");
	ros::NodeHandle nh;
	GPS GPS(nh);
	running =1;
	signal(SIGINT, MySigintHandler);
	ros::Rate loop_rate(1);  //esc need 50 Hz
	while(ros::ok()&running)
	{
		GPS.read_serial();
		ros::spinOnce();
	}
	return 0;
}
