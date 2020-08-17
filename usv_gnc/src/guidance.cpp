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

Guidance::Guidance()
{
	ros::NodeHandle nh;
	ros::NodeHandle node_priv("~");
	node_priv.getParam("lat", lat);//latitude array of the waypoints
	node_priv.getParam("lon", lon);//longitude array of the waypoints
	ROS_INFO("lat[%f,%f]", lat[0],lat[1]);
	ROS_INFO("lon[%f,%f]", lon[0],lon[1]);
	pre_point.x=0.0;
	pre_point.y=0.0;
	pre_point.z=0.0;  
	ros::Subscriber sub1 = nh.subscribe("/sensor/m_fix", 5, &Guidance::getgps, this);
	ros::Subscriber sub2 = nh.subscribe("/sensor/imu/data", 5, &Guidance::getimu, this);
	m_controllerPub =nh.advertise<std_msgs::Float64>("/gnc/c_heading", 5);
	//pubstop =nh.advertise<std_msgs::Int8>("stop", 5);
	pubnextwp =nh.advertise<geometry_msgs::Point32>("/gnc/next_wp", 5);
	ros::Rate rate(10.0);
	while (nh.ok())
	{
    // Get the pose of the robot in the map frame
    ros::spinOnce();
	if(x!=0.0&&y!=0.0&&start==0)
	{
		pre_point.x=x;
		pre_point.y=y;
		start=1;
	}
	if(start==1)
	{
		followPath(x, y, psi);
	}//x,y is the position in map frame.
	ROS_INFO("pre_point[%f,%f]", pre_point.x,pre_point.y);
	ROS_INFO("current[%f,%f]", x,y);
    rate.sleep();
	}
}

Guidance::~Guidance() {}

void Guidance::getgps(const sensor_msgs::NavSatFix & fix) 
{ 
    LLtoUTM(1, fix.latitude, fix.longitude, N, E, Z);  // get utm coordinate
	x=E;
	y=N;
}

void Guidance::getimu(const sensor_msgs::Imu::ConstPtr& msg) 
{ 
		tf::Quaternion q(msg->orientation.x,
			 msg->orientation.y,
			 msg->orientation.z,
			 msg->orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(rpy_data.x, rpy_data.y, rpy_data.z); ///RPY in rad
		psi= rpy_data.z*180/3.1415;
}

void Guidance::followPath(double x, double y, double psi)

{
	int num=lat.size();   //number of waypoints
	if ( num<= 1)
	{
		return;
	}
	if (index ==num)
	{
		index=0;
	}
	LLtoUTM(1, lat[index], lon[index], N, E, Z);
	next_point.x = E;
	next_point.y = N;
	double dist = std::sqrt(std::pow(x - next_point.x, 2) +
                            std::pow(y - next_point.y, 2));
	if (dist<THRESHOULD_D){     
		index++;
		pre_point=next_point;
		LLtoUTM(1, lat[index], lon[index], N, E, Z);
		next_point.x = E;
		next_point.y = N;
	}
    //ROS_INFO("num,index[%d,%d]", num,index);
	ROS_INFO("next[%f,%f]", next_point.x,next_point.y);
	UTMtoLL(1, next_point.y, next_point.x, Z, Lat, Long);
	gpsnext_point.x=Lat;
	gpsnext_point.y=Long;
	pubnextwp.publish(gpsnext_point);
	//the following is LOS
	double dx=next_point.x-pre_point.x;
	double dy=next_point.y-pre_point.y;
	double ddx=next_point.x-x;
	double ddy=next_point.y-y;
	double l_ag = std::atan2(dy,dx);// dx dy 
	double et=-std::sin(l_ag)*ddx + std::cos(l_ag)*ddy;
	double c_heading = std::atan2(dy,dx) - std::atan(-et/DENO); //+ - ddy ddx
	c_heading = c_heading*180/3.1415;
	if(c_heading<-180)
	{
		c_heading=c_heading+360;
	}
		if(c_heading>180)
	{
		c_heading=c_heading-360;
	}
	std_msgs::Float64 msg;
	msg.data = c_heading;
	m_controllerPub.publish(msg);
}
