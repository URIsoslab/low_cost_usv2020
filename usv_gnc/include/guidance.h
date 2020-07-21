#ifndef OTTER_GUIDANCE_H_
#define OTTER_GUIDANCE_H_
#define THRESHOULD_D 10 //threshould to comfirm a waypoint is reached
#define DENO 15 //c_heading = std::atan2(ddy,ddx) - std::atan(-et/DENO);
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/NavSatFix.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64.h"
#include <tf/tf.h>
#include "geometry_msgs/Vector3.h"
class Guidance
{
public:
  Guidance();
  ~Guidance();

private:
  void followPath(double x, double y, double psi);
  void getgps(const sensor_msgs::NavSatFix & gps);
  void getimu(const sensor_msgs::Imu::ConstPtr& msg);
  //double dist(double x0, double y0, double x1, double y1) const;
  double x=0.0, y=0.0, psi=0.0;
  //geometry_msgs::PolygonStamped m_path;
  geometry_msgs::Point32 next_point;
  geometry_msgs::Point32 gpsnext_point;
  geometry_msgs::Point32 pre_point;
  ros::Publisher m_controllerPub,pubstop,pubnextwp;
	std::vector<double> lat;
	std::vector<double> lon;
geometry_msgs::Vector3 rpy_data;
  // lookahead distance
  ///double DELTA = 0.5;

  // time-varying lookahead distance
 // double delta_max = 4.0;
 // double delta_min = 1.0;
 // double delta_k = 1.0;

  // circle of acceptance
 // double R = 1.0;
  int index =0; //index of waypoints
  int start=0;
  //double m_turnSpeed;
  //double m_navSpeed;
};


#endif
