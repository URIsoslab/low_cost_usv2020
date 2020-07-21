/*
Author Mingxi Zhou, mzhou@uri.edu
This driver is created to convert between common ros messages into nmea0183
for serial communication.
The code was made for Digi RF Xbee pro 900Mhz module.
*/
#pragma once
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <cstdio>
#include <ctype.h>
#include <stdint.h>
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "gps_common/GPSFix.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/TimeReference.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/BatteryState.h"


#define DEFAULT_PORT "/dev/ttyUSB0"    // Default port name
#define DEFAULT_BAUD 115200              // Default baudate
#define DEFAULT_PARITY 'N'               // Default parity
#define DEFAULT_DATABITS 8               // Default databits
#define DEFAULT_STOPBITS 1               // Default stopbits
#define DEFAULT_TIMEOUT 1000             // Default timeout

#define FOO_MSG_ID 0
#define IMU_MSG_ID 1
#define GPS_MSG_ID 2
#define ODO_MSG_ID 3
#define MOT_MSG_ID 4
#define SYS_MSG_ID 5
#define CTR_MSG_ID 6
#define JOY_MSG_ID 7


//struct variables for diffferent msg id.
// please refer to readme.md for the msg id explaination
struct IMU_value
{
  double ang_vel[3];
  double lin_accel[3];
  double ori[4];
  double time;
  char frame[20];
  int num=12;
};

struct GPS_value
{
		char frame[20];
		double time;
		double latitude;
		double longitude;
		double sog;
		double cog;
    double roll;
    double pitch;
		double utc_time;
    int num = 9;
};

struct JOY_value
{
  double time;
  double ls;
  double rs;
  int st; //start;
  int bk; //back
  int A;
  int B;
  int X;
  int Y;
  int num = 9;
};

struct CTR_value
{
  int mode;
  double stbd_dc;
  double port_dc;
  double c_heading;
  double c_wptx;
  double c_wpty;
  int num = 6;
};

struct SYS_value
{
  double time;
  float vm;
  float im;
  float vp;
  float ip;
  int num=5;
};

class RF
{
	std::string _s = "$FOO,10,20,30,40*42\r\n";
	int rf_port;
	int N2R_DEFAULT=0;	//control what to convert from nmea to rostopic
	int R2N_DEFAULT=0;	//control what to convert from ros to nmea
  std::string port_name="/dev/ttyS0";
	char MSG_UPDATE_FLAG[10]={0}; 	//bit indicate the message update status
  //NMEA char arrays
	char imu_data[256];	// /imu/data msg in char array
	char gps_data[256];
  char joy_data[256];
  char sys_data[256];

	float hz=20;
	GPS_value gps_val;
  IMU_value imu_val;
  JOY_value joy_val;
  SYS_value sys_val;
  int running=0;


  public:
  RF(ros::NodeHandle nh);
  //imu to nmea
  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
  //GPS to nmea
  void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void time_callback(const sensor_msgs::TimeReference::ConstPtr& msg);
  void rpy_callback(const geometry_msgs::Vector3::ConstPtr& msg);
  //joy to nmea
  void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
  //VI messages to nmea0183
  void vim_callback(const sensor_msgs::BatteryState::ConstPtr& msg);
  void vip_callback(const sensor_msgs::BatteryState::ConstPtr& msg);
  //nmea tx callback
  void rf_tx_callback(const std_msgs::Int32::ConstPtr& msg);
  void NMEA2ROS();
  int parse_imu(char data[256]);
  int parse_gps(char data[256]);
  int parse_joy(char data[256]);
  int parse_sys(char data[256]);
  bool calcChecksum(int &crc, char cdata[256]);
  bool checkIntegrity(std::string& nmea_data);
  private:
    //foo template
  	ros::Publisher foo_pub;
  	ros::Subscriber foo_sub;
  	//imu receiver and publisher
  	ros::Subscriber imu_sub;		//subscribe to imu message for rf nmea msg=1
  	ros::Subscriber fix_sub;		//subscribe to imu message for rf nmea msg=1
  	ros::Subscriber vel_sub;	//hear beat for rf tx
    ros::Subscriber rpy_sub;
  	ros::Subscriber time_sub;
    ros::Subscriber joy_sub;
    ros::Subscriber vim_sub;
    ros::Subscriber vip_sub;
    //nmea trigger publisher
    ros::Publisher imu_pub;			// publish imu data based on nmea0183 string
  	ros::Publisher gps_pub;			// publish imu data based on nmea0183 string
    ros::Publisher joy_pub;
    ros::Publisher vm_pub, vp_pub;
  	ros::Subscriber rf_tx_beat;// controls rf tx frequency
  	//serial port
  	serial::Serial *ser;


};
