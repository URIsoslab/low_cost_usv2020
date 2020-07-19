#include <digi_rf/rf_driver.h>

RF::RF(ros::NodeHandle nh)
{
	//initial publishers and subscribers
	//initalize the port

	// initialize subscriber and publisher based on rospara
	ros::NodeHandle nh_priv("~");
	nh_priv.getParam("NMEA2ROS",N2R_DEFAULT);
	nh_priv.getParam("ROS2NMEA",R2N_DEFAULT);
	nh_priv.getParam("port_name", port_name);

  signal(SIGINT, RF::MySigintHandler);
	ser = new serial::Serial(port_name,115200, serial::Timeout::simpleTimeout(1000));
	ser->flushInput();
	//subscribe to IMU/data topic
	rf_tx_beat = nh.subscribe("/rf/beat",2,&RF::rf_tx_callback,this);
	imu_sub = nh.subscribe("/imu/data",2, &RF::imu_callback,this);
	//subscribe to nmea gps_val topics
	fix_sub = nh.subscribe("/fix",2, &RF::fix_callback,this);
	vel_sub = nh.subscribe("/vel",2, &RF::vel_callback,this);
	time_sub = nh.subscribe("/time_reference",2, &RF::time_callback,this);
	//publishers
	imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 2);
	gps_pub = nh.advertise<gps_common::GPSFix>("/gps",2);
}

void RF::MySigintHandler(int sig)
{
	//shut things down
	//running=0;
	return;
}
///add callback functions here to generate rf messages//
/////////////////////////////////////////////////////
///msg ID =1//// IMU topic convert to nmea0183
void RF::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	char o_data[256];
	int cs;
	int num;
	strcpy(imu_val.frame,msg->header.frame_id.c_str());
	imu_val.time = msg->header.stamp.toSec();
	imu_val.ori[0]=msg->orientation.x;
	imu_val.ori[1]=msg->orientation.y;
	imu_val.ori[2]=msg->orientation.z;
	imu_val.ori[3]=msg->orientation.w;
	imu_val.ang_vel[0]=msg->angular_velocity.x;
	imu_val.ang_vel[1]=msg->angular_velocity.y;
	imu_val.ang_vel[2]=msg->angular_velocity.z;
	imu_val.lin_accel[0]=msg->linear_acceleration.x;
	imu_val.lin_accel[1]=msg->linear_acceleration.y;
	imu_val.lin_accel[2]=msg->linear_acceleration.z;
	//01| $IMU,frame,time,ori.x,ori.y,ori.z,orientation.w,ang_v.x,ang_v.y,ang_v.z,l_v.x,l_v.y,l_v.z*cs\r\n
	num = sprintf(o_data,"$IMU,%s,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*00\r\n",
																			imu_val.frame, imu_val.time,
																			imu_val.ori[0],imu_val.ori[1],imu_val.ori[2],imu_val.ori[3],
																			imu_val.ang_vel[0],imu_val.ang_vel[1],imu_val.ang_vel[2],
																			imu_val.lin_accel[0],imu_val.lin_accel[1],imu_val.lin_accel[2]);
	std::string o_s=o_data;
//	ROS_INFO("%s",o_data);
	calcChecksum(cs,o_data);
	//re-do the sprintf to imu_data
	num = sprintf(imu_data,"$IMU,%s,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*%x\r\n",
																	imu_val.frame, imu_val.time,
																	imu_val.ori[0],imu_val.ori[1],imu_val.ori[2],imu_val.ori[3],
																	imu_val.ang_vel[0],imu_val.ang_vel[1],imu_val.ang_vel[2],
																	imu_val.lin_accel[0],imu_val.lin_accel[1],imu_val.lin_accel[2],cs);
	//ser->write(o_data);
	MSG_UPDATE_FLAG[IMU_MSG_ID] = 1; //SET THE FLAG
}

//msg ID 2 GPS
//GPS callback
	//MSG ID =2
void RF::fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	strcpy(gps_val.frame,msg->header.frame_id.c_str());
	gps_val.latitude=msg->latitude;
	gps_val.longitude=msg->longitude;
	gps_val.time = msg->header.stamp.toSec();
}
void RF::vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	gps_val.sog = pow(msg->twist.linear.x,2) + pow(msg->twist.linear.x,2); //m/s
	gps_val.cog = atan2(msg->twist.linear.x, msg->twist.linear.y)*180/3.1415926; //deg
}

void RF::time_callback(const sensor_msgs::TimeReference::ConstPtr& msg)
{
	char o_data[256];
	int num;
	int cs;
	gps_val.utc_time=msg->time_ref.toSec();
	num=sprintf(o_data,"$GPS,%s,%.3lf,%.5lf,%.5lf,%.3lf,%.3lf,%.3lf*00\r\n",
														 gps_val.frame,gps_val.time,gps_val.latitude,gps_val.longitude,
													   gps_val.cog,gps_val.sog,gps_val.utc_time);
	calcChecksum(cs,o_data);
	num=sprintf(gps_data,"$GPS,%s,%.3lf,%.5lf,%.5lf,%.3lf,%.3lf,%.3lf*%X\r\n",
														 gps_val.frame,gps_val.time,gps_val.latitude,gps_val.longitude,
													   gps_val.cog,gps_val.sog,gps_val.utc_time,cs);
  MSG_UPDATE_FLAG[GPS_MSG_ID] = 1; //SET THE FLAG
}

///////rf tx callback
void RF::rf_tx_callback(const std_msgs::Int32::ConstPtr& msg)
{
			if(MSG_UPDATE_FLAG[IMU_MSG_ID])
			{
				ser->write(imu_data); //send the data
				MSG_UPDATE_FLAG[IMU_MSG_ID]=0; //reset the flag
			}
			if(MSG_UPDATE_FLAG[GPS_MSG_ID])
			{
				ser->write(gps_data); //send the data
				MSG_UPDATE_FLAG[GPS_MSG_ID]=0; //reset the flag
			}
}

////////////////checksum calculation//////////////////////////
bool RF::calcChecksum(int &crc, char cdata[256])	//return check flag and return actual crc in the bracket
{
    	//NMNA0183
			std::string ss = cdata;
    	int sz = ss.size();
    	int i;
			crc=0;
    	//$*00<CR><LF>
    	for (i = 1; i < sz - 5; i ++) {
				crc ^= cdata[i];
    	}
			int checksum =std::stoi(ss.substr(sz-4, 2), 0, 16);
  		return crc == checksum;
}
