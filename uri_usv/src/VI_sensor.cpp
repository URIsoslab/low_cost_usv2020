/**
 This program talks to the VI sensor then publish the voltage and current reading.
 Jan-09-2020 Mingxi, clean up program rename nodes and topics
 01-14-2020 Jianguang, output power consumption from the battery charged in Wh and store it to a txt file.
If the batteries are just fully charged, set the chargeFlag parameter to be 1. Otherwise set to 0.
 */
 //include c language libraries
 extern "C" {
 #include "rc_usefulincludes.h"
 }
 extern "C" {
 #include "roboticscape.h"
 }
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
#include <rc/i2c.h>
#include <rc/time.h>
//include ros stuff
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include<sensor_msgs/BatteryState.h>
//define Sensor registers
#define INA260_I2CADDR_DEFAULT 0x40  ///< INA260 default i2c address
#define INA260_REG_CURRENT 0x01      ///< Current measurement register (signed) in mA
#define INA260_REG_BUSVOLTAGE 0x02   ///< Bus voltage measurement register in mV
#define INA260_REG_POWER        0x03 ///< Power calculation register in mW
#define INA260_REG_MASK_ENABLE 0x06  ///< Interrupt/Alert setting and checking register
#define INA260_REG_ALERT_LIMIT 0x07  ///< Alert limit value register
#define INA260_REG_MFG_UID 0xFE      ///< Manufacturer ID Register
#define INA260_REG_DIE_UID 0xFF      ///< Die ID and Revision Register

using namespace std;
ofstream pcsout;  //power consumption out
ifstream pcsin;  //power consumption in
sensor_msgs::BatteryState batt_state;
void MySigintHandler(int sig)
{
  //close the program
  ROS_INFO("shutting down!");
  pcsout<<batt_state.charge<<"\n";
  rc_i2c_close(1);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  //  uint8_t re[17];
  // initialize a ros node (name used in ros context)
  uint8_t cu[2], vo[2], po[2];
  int chargeFlag;
  float power;
  batt_state.charge=0.0;   //batt_state.charge is actually the energy consumed in Ah
  ros::init(argc, argv, "VI_sensor");
  // create a handle of the node (name used in the file)
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");
  node_priv.param("chargeFlag", chargeFlag, 0);
  if(chargeFlag==0){
	  pcsin.open("/home/ubuntu/catkin_ws/src/uri_usv/files/parameters.txt");
	  pcsin>>batt_state.charge;
	  pcsin.close();
  }
  pcsout.open("/home/ubuntu/catkin_ws/src/uri_usv/files/parameters.txt",ios::trunc);
  signal(SIGINT, MySigintHandler);
  // create a Publisher and publish a string topic named heading
  ros::Publisher batt_pub = nh.advertise<sensor_msgs::BatteryState>("/sensor/m_VI", 2);
  // set the frequency. It should be conbined with spinOnce().
  ros::Rate loop_rate(1);
  //init i2c
  rc_i2c_init(1, INA260_I2CADDR_DEFAULT);
  // get the voltage and current data and publish it by the publisher
  while (ros::ok())
  {
    //// get the voltage and current data and publish it by the publisher
    rc_i2c_read_bytes(1, INA260_REG_CURRENT, 2, cu);
    // rc_i2c_set_device_address(1, INA260_I2CADDR_DEFAULT);
    rc_i2c_read_bytes(1, INA260_REG_BUSVOLTAGE, 2, vo);
    rc_i2c_read_bytes(1, INA260_REG_POWER, 2, po);
	//ROS_INFO("voltage0:%d\n voltage1:%d\n", int(vo[0]),int(vo[1]));
    
	  batt_state.voltage = 1.25*int((vo[0]<<8)|(vo[1]))/1000; //combine the high bits and the low bits;
	  batt_state.current = 1.25*int((cu[0]<<8)|(cu[1]))/1000;
    power=10*int((po[0]<<8)|(po[1]))/1000;
	batt_state.charge=batt_state.charge+power*1.00/3600.00;
	//node_priv.setParam("Wh", batt_state.charge);
    batt_pub.publish(batt_state);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
