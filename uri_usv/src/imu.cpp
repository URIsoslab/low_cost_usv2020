/*
################################################################################
Jan-09-2020, Mingxi, Code clean up, and remove the rpm averaging
                     changed topic name with standard prefix
                     m_imu, m_rpy, m_heading
*/
//include C language libraries
//extern "C" {
//#include "rc_usefulincludes.h"
//}
extern "C" {
#include "roboticscape.h"
}

#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
//#include <rc/mpu.h>
//#include <rc/time.h>
#include <sstream>

#include "ros/ros.h"
//include ros message types
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
//define I2C port for the IMU
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21

//global variables
static int running = 0;
/**
 * @brief      interrupt handler to catch ctrl-c
 */
void MySigintHandler(int sig)
{
  //shut things down
  rc_mpu_power_off();
  fflush(stdout);
  running=0;
  return;
}

int main(int argc, char** argv)
{
	// initialize a ros node (name used in ros context)
	ros::init(argc, argv, "imu");
	// create a handle of the node (name used in the file)
	ros::NodeHandle nh;
 	// create a Publisher and publish a string topic named heading
  	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/sensor/imu/m_raw", 2);
  	ros::Publisher rpy_pub = nh.advertise<geometry_msgs::Vector3>("/sensor/imu/m_rpy", 2);
//  ros::Publisher heading_pub = nh.advertise<std_msgs::Float64>("/sensor/imu/m_heading", 2);
  	// set the frequency. It should be conbined with spinOnce().
  	ros::Rate loop_rate(10);

  	////configuration of the imu
	rc_mpu_data_t data;
  	rc_mpu_config_t conf = rc_mpu_default_config();
  	conf.i2c_bus = I2C_BUS;
  	conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
	conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
	//IMU Configuration setting
  	conf.dmp_sample_rate = 100;	//sampling rate
  	conf.enable_magnetometer = 1;	//enable magnetometer, set to 0 for no magnetometer
	conf.dmp_auto_calibrate_gyro = 1;	//enable auto calibrate on the gyro
	conf.dmp_fetch_accel_gyro=1;		//enable accel and gyro reading in dmp.
	//refer to the robotic control library for more options
	  // initialize the imu
  	if (rc_mpu_initialize_dmp(&data, conf))
 	{
    	printf("rc_initialize_imu_failed\n");
    	return -1;
  	}
  //if ctrl-c we shut down things
  	signal(SIGINT, MySigintHandler);
  	running = 1;
	// data define
	sensor_msgs::Imu imu_data;   //IMU data
	geometry_msgs::Vector3 rpy_data; //roll pitch yaw data
  	std_msgs::Float64 heading_data;  //heading data
	ROS_INFO("Node started");
	while (ros::ok()&running)
	{
  	rpy_data.x = data.fused_TaitBryan[TB_PITCH_X] * RAD_TO_DEG;
    	rpy_data.y = data.fused_TaitBryan[TB_ROLL_Y] * RAD_TO_DEG;
    	rpy_data.z = data.fused_TaitBryan[TB_YAW_Z] * RAD_TO_DEG;
    	imu_data.linear_acceleration.x = data.accel[0];
    	imu_data.linear_acceleration.y = data.accel[1];
    	imu_data.linear_acceleration.z = data.accel[2];
    	imu_data.angular_velocity.x = data.gyro[0];
    	imu_data.angular_velocity.y = data.gyro[1];
    	imu_data.angular_velocity.z = data.gyro[2];
    	//heading_data.data = data.compass_heading ;//the control input for pid should be rad
    	imu_pub.publish(imu_data);
    	rpy_pub.publish(rpy_data);
  //  heading_pub.publish(heading_data);
    		//// get the heading data and publish it by the publisher
    		// circulate at the set rate
  	ros::spinOnce();
  	loop_rate.sleep();
	}

  	return 0;
}
