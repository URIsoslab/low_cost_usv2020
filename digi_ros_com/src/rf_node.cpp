#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <cstdio>
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"

#define DEFAULT_BAUD 115200              // Default baudate
#define DEFAULT_PARITY 'N'               // Default parity
#define DEFAULT_DATABITS 8               // Default databits
#define DEFAULT_STOPBITS 1               // Default stopbits
#define DEFAULT_TIMEOUT 1000             // Default timeout
int running=1;
///control-c control
void MySigintHandler(int sig)
{
	//shut things down
	running=0;
	return;
}



class RF
{
	std::string _s = "$FOO,10,20,30,40*42\r\n";
	char _data[256];
	char header[3];
	char msg_ID=0;
	double val[10];
	int flag = 0;
	int _cs=0;
	int rf_port;
	int N2R_DEFAULT=0;
	int R2N_DEFAULT=0;
	char imu_data[256];
	float hz=1;
//  serial::Serial ser(''/dev/ttyUSB0',115200, serial::Timeout::simpleTimeout(1000));
	public:
	RF()
	{
		//initial publishers and subscribers
		// control-c detection
		signal(SIGINT, MySigintHandler);
		//initalize the port
		ser = new serial::Serial("/dev/ttyS1",115200, serial::Timeout::simpleTimeout(1000));
		ser->flushInput();
		// initialize subscriber and publisher based on rospara
		ros::NodeHandle nh_priv("~");
		nh_priv.getParam("NMEA2ROS",N2R_DEFAULT);
		nh_priv.getParam("ROS2NMEA",R2N_DEFAULT);
		//subscribe to IMU/data topic
		imu_sub = nh.subscribe("/imu/data",2, &RF::imu_callback,this);

	}
	///add callback functions here to generate rf messages//
	/////////////////////////////////////////////////////
	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		char o_data[256];
		int cs;
		double ang_vel[3], lin_vel[3], ori[4], time;
		char frame[20];
		strcpy(frame,msg->header.frame_id.c_str());
		time = msg->header.stamp.toSec();
		ori[0]=msg->orientation.x;
		ori[1]=msg->orientation.y;
		ori[2]=msg->orientation.z;
		ori[3]=msg->orientation.w;
		ang_vel[0]=msg->angular_velocity.x;
		ang_vel[1]=msg->angular_velocity.y;
		ang_vel[2]=msg->angular_velocity.z;
		lin_vel[0]=msg->linear_acceleration.x;
		lin_vel[1]=msg->linear_acceleration.y;
		lin_vel[2]=msg->linear_acceleration.z;
		//01| $IMU,frame,time,ori.x,ori.y,ori.z,orientation.w,ang_v.x,ang_v.y,ang_v.z,l_v.x,l_v.y,l_v.z*cs\r\n
		int num = sprintf(o_data,"$IMU,%s,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*00\r\n",
																				frame,time,
																				ori[0],ori[1],ori[2],ori[3],
																				ang_vel[0],ang_vel[1],ang_vel[2],
																				lin_vel[0],lin_vel[1],lin_vel[2]);
		std::string o_s=o_data;
		ROS_INFO("%s",o_data);

  	calcChecksum(cs,o_data);
		//re-do the sprintf to imu_data
		num = sprintf(imu_data,"$IMU,%s,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*%d\r\n",
																				frame,time,
																				ori[0],ori[1],ori[2],ori[3],
																				ang_vel[0],ang_vel[1],ang_vel[2],
																				lin_vel[0],lin_vel[1],lin_vel[2],cs);
		//ser->write(o_data);
	}
	///this program subscribe to its own message to control the rf send frequency

	//parse the data
	void NMEA2ROS()
	{
		std::string s_data;
		std::size_t sz_t = 120;
    std::string eol;
    eol.append(1,0xd);
    eol.append(1,0xa);
		int num;
		while(1&running)
		{
			//check serial port for data
		//	s_data.clear();
    //  if(ser->available()){
    //        s_data = ser->readline(sz_t,eol);
		//	}
			//convert string to char
			strcpy(_data,s_data.c_str());
			//check the first byte
			if(_data[0]=='$')
			{
				//get header
				header[0] = _data[1];
				header[1] = _data[2];
				header[2] = _data[3];
				///////foo template//////////
				char temp[3]={'F','O','O'};
				if(strcmp(temp,header)==0)
				{
					msg_ID=1;
				}
				//add more msgs
				/////////////////////////////
				if(N2R_DEFAULT & (0x01>>(msg_ID-1)) )
				{
					switch(msg_ID)
					{
						case 1: //foo
								num = sscanf(_data,"$FOO,%lf,%lf,%lf,%lf*\r\n",
										       &val[0], &val[1], &val[2], &val[3]);
								//if num of field is right and the checksum is right
								if(num==4&&calcChecksum(_cs,_data))
								{
									flag=1;
									ROS_INFO("val1=%lf",val[0]);
									//convert the vals into ros messages
								}
							break;
						//add more cases here
						default:
							break;
					}//end of switch
				}// end of if
			}//end of first byte check
		}//end of while(1)
	}//end of function

	/////////////////////////////////////////////////////
	bool calcChecksum(int &crc, char cdata[256])	//return check flag and return actual crc in the bracket
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

	void rf_running()
	{
		ros::Rate loop_rate(hz);
		running=1;
		while (ros::ok()&running)
		{
		//		ROS_INFO("running");
				ser->write(imu_data);
				ros::spinOnce();
				loop_rate.sleep();
		}

	}

	private:
	ros::NodeHandle nh;
	ros::Publisher foo_pub;
	ros::Subscriber foo_sub;
	ros::Subscriber imu_sub;
	serial::Serial *ser;
};


int main (int argc, char **argv)
{
	ros::init(argc, argv, "rf_node");
	RF RF_node;
//	RF_node.NMEA2ROS();
  RF_node.rf_running();

	return 0;
}
