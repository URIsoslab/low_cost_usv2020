/*receive waypoints info from paramter
go to way [0] from start point by LOS;
when arrived to the last waypoint, return to the first waypoint

publish /c_heading
publish /stop

subscribe gps and  heading
*/

#include <guidance/guidance.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <guidance/utmll.h>
//namespace otter_coverage
int main(int argc, char** argv)
{
	std::vector<double> lat;
	std::vector<double> lon;
  ros::init(argc, argv, "utm");
  ros::NodeHandle n;
  ros::NodeHandle node_priv("~");
	node_priv.getParam("lat", lat);//latitude array of the waypoints
	node_priv.getParam("lon", lon);//longitude array of the waypoints
for(int index=0;index<lat.size();index++)
{
	LLtoUTM(1, lat[index], lon[index], N, E, Z);
	double x = E;
	double y = N;
    ROS_INFO("index %d : %f,%f", index,x,y);
  }

  return 0;
}

