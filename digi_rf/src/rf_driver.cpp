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

	ser = new serial::Serial(port_name,115200, serial::Timeout::simpleTimeout(1000));
//	ser->flushInput();
	ser->flush();
	rf_tx_beat = nh.subscribe("/rf/beat",2,&RF::rf_tx_callback,this);
	//subscribe to IMU/data topic
	if(R2N_DEFAULT>>(IMU_MSG_ID-1) & 0x01)
	{
	ROS_INFO("R2N IMU");
	imu_sub = nh.subscribe("/sensor/imu/data",2, &RF::imu_callback,this);
	}
	//subscribe to nmea gps_val topics
	if(R2N_DEFAULT>>(GPS_MSG_ID-1) & 0x01)
	{
	ROS_INFO("R2N GPS");
	fix_sub = nh.subscribe("/sensor/m_fix",2, &RF::fix_callback,this);
	vel_sub = nh.subscribe("/sensor/m_vel",2, &RF::vel_callback,this);
	time_sub = nh.subscribe("/sensor/m_time_reference",2, &RF::time_callback,this);
	rpy_sub = nh.subscribe("/sensor/imu/rpy",2, &RF::rpy_callback,this);
	}
	if(R2N_DEFAULT>>(SYS_MSG_ID-1) & 0x01)
	{
	ROS_INFO("R2N SYS");
	vim_sub = nh.subscribe("/sensor/m_VI",2,&RF::vim_callback,this);
	vip_sub = nh.subscribe("/sensor/m_VI_payload",2,&RF::vip_callback,this);
	}
	//subscribe to joy then convert to nmea0183
	if(R2N_DEFAULT>>(JOY_MSG_ID-1) & 0x01)
	{
	ROS_INFO("R2N JOY");
	joy_sub = nh.subscribe("/joy",2,&RF::joy_callback,this);
	}
	//publishers/////////////////////
	if(N2R_DEFAULT>>(IMU_MSG_ID-1) & 0x01 )
	{
	ROS_INFO("N2R IMU");
	imu_pub = nh.advertise<sensor_msgs::Imu>("/rf/sensor/imu/data", 2);
	}
	if(N2R_DEFAULT>>(GPS_MSG_ID-1) & 0x01 )
	{
	ROS_INFO("N2R GPS");
	gps_pub = nh.advertise<gps_common::GPSFix>("/rf/sensor/gps",2);
	}
	if(N2R_DEFAULT>>(JOY_MSG_ID-1) & 0x01 )
	{
	ROS_INFO("N2R JOY");
	joy_pub = nh.advertise<sensor_msgs::Joy>("/rf/joy",2);
	}
	if(N2R_DEFAULT>>(SYS_MSG_ID-1) & 0x01 )
	{
	ROS_INFO("N2R SYS");
	vm_pub  = nh.advertise<sensor_msgs::BatteryState>("/rf/sensor/m_VI",2);
	vp_pub  = nh.advertise<sensor_msgs::BatteryState>("/rf/sensor/m_VI_payload",2);
	}
}


///Converting ROS messages into nmea0183
////////////////////////////////////////////////////////////////
///msg ID =1//// IMU topic convert to nmea0183//////////////////
/////////////////////////////////////////////////////////
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
	num = sprintf(imu_data,"$IMU,%s,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*%02X\r\n",
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

void RF::rpy_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	gps_val.roll=msg->x;
	gps_val.pitch=msg->y;
}

void RF::time_callback(const sensor_msgs::TimeReference::ConstPtr& msg)
{
	char o_data[256];
	int num;
	int cs;
	gps_val.utc_time=msg->time_ref.toSec();
	num=sprintf(o_data,"$GPS,%s,%.3lf,%.5lf,%.5lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*00\r\n",
														 gps_val.frame,gps_val.time,gps_val.latitude,gps_val.longitude,
													   gps_val.cog,gps_val.sog,
														 gps_val.roll,gps_val.pitch,gps_val.utc_time);
	calcChecksum(cs,o_data);
	num=sprintf(gps_data,"$GPS,%s,%.3lf,%.5lf,%.5lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*%02X\r\n",
														 gps_val.frame,gps_val.time,gps_val.latitude,gps_val.longitude,
													   gps_val.cog,gps_val.sog,
														 gps_val.roll,gps_val.pitch,gps_val.utc_time,cs);
  MSG_UPDATE_FLAG[GPS_MSG_ID] = 1; //SET THE FLAG
}
///joy callback
void RF::joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
	char o_data[256];
	int num;
	int cs;
	joy_val.time = msg->header.stamp.toSec();
	joy_val.ls   = msg->axes[1];
	joy_val.rs   = msg->axes[3];
	joy_val.A    = msg->buttons[1];
	joy_val.B		 = msg->buttons[2];
	joy_val.X    = msg->buttons[0];
	joy_val.Y		 = msg->buttons[4];
	joy_val.st   = msg->buttons[9];
	joy_val.bk	 = msg->buttons[8];
	num=sprintf(o_data,"$JOY,%.3lf,%.3lf,%.3lf,%d,%d,%d,%d,%d,%d*00\r\n",
													joy_val.time, joy_val.ls, joy_val.rs,
													joy_val.st, joy_val.bk, joy_val.A, joy_val.B, joy_val.X, joy_val.Y);
	calcChecksum(cs,o_data);
	num=sprintf(joy_data,"$JOY,%.3lf,%.3lf,%.3lf,%d,%d,%d,%d,%d,%d*%02X\r\n",
													joy_val.time, joy_val.ls, joy_val.rs,
													joy_val.st, joy_val.bk, joy_val.A, joy_val.B, joy_val.X, joy_val.Y,cs);
	MSG_UPDATE_FLAG[JOY_MSG_ID]= 1;
}

///VI sensor callbacks
void RF::vim_callback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	sys_val.time = msg->header.stamp.toSec();
	sys_val.vm   = msg->voltage;
	sys_val.im = msg->current;
}
void RF::vip_callback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	char o_data[256];
	int num;
	int cs;

	sys_val.time = msg->header.stamp.toSec();
	sys_val.vp   = msg->voltage;
	sys_val.ip = msg->current;
	num=sprintf(o_data,"$SYS,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*00\r\n",
													sys_val.time, sys_val.vm, sys_val.im, sys_val.vp, sys_val.ip);
	calcChecksum(cs,o_data);
	num=sprintf(sys_data,"$SYS,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf*%02X\r\n",
													sys_val.time, sys_val.vm, sys_val.im, sys_val.vp, sys_val.ip,cs);
	MSG_UPDATE_FLAG[SYS_MSG_ID]= 1;
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
			if(MSG_UPDATE_FLAG[JOY_MSG_ID])
			{
				ser->write(joy_data); //send the data
				ROS_INFO("%s",joy_data);
				MSG_UPDATE_FLAG[JOY_MSG_ID]=0; //reset the flag
			}
			if(MSG_UPDATE_FLAG[SYS_MSG_ID])
			{
				ser->write(sys_data); //send the data
				MSG_UPDATE_FLAG[SYS_MSG_ID]=0; //reset the flag
			}
}


/////////////////////////////////////////////////////////////////////////////
////Parsing nmea0183 nav_msgs////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
//parse the data
void RF::NMEA2ROS()
{
	int msg_ID=0;
	std::size_t sz_t = 256;
  std::string eol;
  eol.append(1,0xd);
  eol.append(1,0xa);
	char _data[256]={0};
	char __data[256]={0};
	char header[3]={0};
	std::string s_data;

	int c =0;
	//check serial port for data
	s_data.clear();
	if(ser->available())
	{
		s_data = ser->readline(sz_t,"\n");
		strcpy(_data,s_data.c_str());
  //  ROS_INFO("%s",_data);
	//we check the starting byte
	//check the first byte
	if(_data[0]=='$')
	{
		//Use header to determine the msg id for later swith
		///////foo template//////////
		if(_data[1]=='F' & _data[2]=='O' & _data[3]=='O')
		{
			msg_ID=FOO_MSG_ID;
		}
		//temp[0]='I'; temp[1]='M'; temp[2]=='U';
		//IMU data
		if(_data[1]=='I' & _data[2]=='M' & _data[3]=='U')
		{
			msg_ID=IMU_MSG_ID;
		}
		if(_data[1]=='G' & _data[2]=='P' & _data[3]=='S')
		{
			msg_ID=GPS_MSG_ID;
		}
		if(_data[1]=='J' & _data[2]=='O' & _data[3]=='Y')
		{
			msg_ID=JOY_MSG_ID;
		}
		if(_data[1]=='S' & _data[2]=='Y' & _data[3]=='S')
		{
			msg_ID=SYS_MSG_ID;
		}
		//add more MSG ID here
		//lets switch the msg_ID
		switch(msg_ID)
		{
			case IMU_MSG_ID :
			{
				if( parse_imu(_data) )
				{
					ROS_INFO("Got IMU");
				}
			}
			break;

			case GPS_MSG_ID :
			{
				if( parse_gps(_data) )
				{
					ROS_INFO("Got GPS");
				}
			}
			break;

			case JOY_MSG_ID:
			{
				if( parse_joy(_data) )
				{
					ROS_INFO("Got joy");
				}
			}
			break;
			case SYS_MSG_ID:
			{
				if( parse_sys(_data) )
				{
					ROS_INFO("Got sys");
				}
			}
			break;

		/*	case 2:
			break;
			case 3:
			break;
			case 4:
			break;
			case 5:
			break;
			case 6:
			break;
			case 7:
			break;*/
		}
	}
}

}

int RF::parse_imu(char data[256])
{
	IMU_value rf_val;
	sensor_msgs::Imu pub_data;
  int cs=0;
	int num = sscanf(data,"$IMU,%[^,],%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf*",
																			rf_val.frame,&rf_val.time,
																			&rf_val.ori[0],&rf_val.ori[1],&rf_val.ori[2],&rf_val.ori[3],
																			&rf_val.ang_vel[0],&rf_val.ang_vel[1],&rf_val.ang_vel[2],
																			&rf_val.lin_accel[0],&rf_val.lin_accel[1],&rf_val.lin_accel[2]);

  //calcChecksum(cs,data);

	if(num == rf_val.num & calcChecksum(cs,data))
	{
		pub_data.header.frame_id=rf_val.frame;
		pub_data.header.stamp=ros::Time(rf_val.time);
		pub_data.orientation.x=rf_val.ori[0];
		pub_data.orientation.y=rf_val.ori[1];
		pub_data.orientation.z=rf_val.ori[2];
		pub_data.orientation.w=rf_val.ori[3];
		pub_data.angular_velocity.x=rf_val.ang_vel[0];
		pub_data.angular_velocity.y=rf_val.ang_vel[1];
		pub_data.angular_velocity.z=rf_val.ang_vel[2];
		pub_data.linear_acceleration.x=rf_val.lin_accel[0];
		pub_data.linear_acceleration.y=rf_val.lin_accel[1];
		pub_data.linear_acceleration.z=rf_val.lin_accel[2];
		imu_pub.publish(pub_data);
		return 1;
	}
	return 0;
}

int RF::parse_gps(char data[256])
{
	GPS_value rf_val;
  int cs=0;
	gps_common::GPSFix pub_data;
	int num = sscanf(data,"$GPS,%[^,],%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf*",
																			rf_val.frame,&rf_val.time,
																			&rf_val.latitude, &rf_val.longitude,
																			&rf_val.cog, &rf_val.sog,
																			&rf_val.roll,&rf_val.pitch,&rf_val.utc_time);
  //calcChecksum(cs,data);
	if(num == rf_val.num & calcChecksum(cs,data))
	{
		pub_data.header.frame_id=rf_val.frame;
		pub_data.header.stamp=ros::Time(rf_val.time);
		pub_data.longitude = rf_val.longitude;
		pub_data.latitude = rf_val.latitude;
		pub_data.track = rf_val.cog;
		pub_data.speed = rf_val.sog;
		pub_data.roll = rf_val.roll;
		pub_data.pitch = rf_val.pitch;
		pub_data.time = rf_val.utc_time;
		gps_pub.publish(pub_data);
		return 1;
	}
	return 0;
}

int RF::parse_joy(char data[256])
{
	JOY_value rf_val;
	sensor_msgs::Joy pub_data;
	pub_data.axes.resize(5);
	pub_data.buttons.resize(11);
	int cs=0;
	int num = sscanf(data,"$JOY,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d*",
																			&rf_val.time, &rf_val.ls, &rf_val.rs,
																			&rf_val.st, &rf_val.bk,
																			&rf_val.A, &rf_val.B, &rf_val.X, &rf_val.Y);

  //calcChecksum(cs,data);
	if(num == rf_val.num & calcChecksum(cs,data))
	{
		pub_data.header.stamp=ros::Time(rf_val.time);
		//pub_data.axes[0]=0;
		pub_data.axes[1]=rf_val.ls;
		//pub_data.axes[2]=0;
		pub_data.axes[3]=rf_val.rs;
		//pub_data.axes[4]=0;
		//pub_data.axes[5]=0;
		pub_data.buttons[0]=rf_val.X;
		pub_data.buttons[1]=rf_val.A;
		pub_data.buttons[2]=rf_val.B;
		//pub_data.buttons[3]=0;
		pub_data.buttons[4]=rf_val.Y;
		//pub_data.buttons[5]=0;
		//pub_data.buttons[6]=0;
		//pub_data.buttons[7]=0;
		pub_data.buttons[8]=rf_val.bk;
		pub_data.buttons[9]=rf_val.st;
		//pub_data.buttons[10]=0;
		//pub_data.buttons[11]=0;
		joy_pub.publish(pub_data);
		return 1;
	}
	return 0;
}

int RF::parse_sys(char data[256])
{
	SYS_value val;
	sensor_msgs::BatteryState vm_data;
	sensor_msgs::BatteryState vp_data;
	int cs=0;
	int num = sscanf(data,"$SYS,%lf,%f,%f,%f,%f*",&val.time, &val.vm, &val.im, &val.vp, &val.ip);
	//calcChecksum(cs,data);
	if(num == val.num & calcChecksum(cs,data))
	{
		vm_data.header.stamp=ros::Time(val.time);
		vp_data.header.stamp=ros::Time(val.time);
		vm_data.voltage = val.vm;
		vm_data.current = val.im;
		vp_data.voltage = val.vp;
		vp_data.current = val.ip;
		vm_pub.publish(vm_data);
		vp_pub.publish(vp_data);
		return 1;
	}
	return 0;

}

////////////////checksum calculation//////////////////////////
bool RF::calcChecksum(int &crc, char cdata[256])	//return check flag and return actual crc in the bracket
{
    	//NMNA0183
			std::string ss = cdata;
    	int sz = ss.size();
		//	ROS_INFO("%s,%d,%c",cdata,sz,cdata[sz-4]);
    	int i;
			//for (i = 0; i < sz; i ++)
			//{
				//printf("%c=%d,",cdata[i],cdata[i]);
			//}
			crc=0;
    	//$*00<CR><LF>
		//	if(!checkIntegrity(ss))
		//	{
			// return 0;
	 		//}
    	for (i = 1; i < sz - 5; i ++)
			{
				crc ^= cdata[i];
    	}

		//	ROS_INFO("end");
		//	int checksum = std::stoi(cs,0,16);
			int checksum =std::stoi(ss.substr(sz-4, 2), nullptr, 16);
  		return crc == checksum;

			return 0;
}




bool RF::checkIntegrity(std::string& nmea_data)
{
    // return error_value

    int sz = nmea_data.size();
    int i;

    // check min size
    if(sz<=6){
        ROS_INFO("bad size");
        return false;
    }

    // find bad char
    bool isFound = false;
    std::vector<int> bad_char;
    for(i=0; i<sz; i++)
    {
        if( (nmea_data[i] < 32 || nmea_data[i] > 126) && nmea_data[i] != 10 && nmea_data[i] != 13){//ASCII: "Space" and "~"
            bad_char.push_back(i);
            isFound = true;
        }
    }
    // delete bad char
    if(isFound){
        for(i=0; i<bad_char.size(); i++)
        {
            nmea_data.erase(bad_char[i]-i,1);
        }

        ROS_WARN("bad char sz is: %ld\n", bad_char.size());
    }

    //$,data,...,data*00<CR><LR>
    //Check: Start delimiter, Checksum delimiter, 2 bits checksum, Carriage return, Line feed
    sz = nmea_data.size();
    if(!(nmea_data.at(0)=='$' && nmea_data.at(sz - 5)=='*')){
        ROS_WARN(" Warning: bad integrality");
        return false;
    }

    return true;
}
