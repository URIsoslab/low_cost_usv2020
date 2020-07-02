/**This program use controls the PWM based on the Joy node
Jan-09-2020 Mingxi This node sets the PWM to two motors
										changed the ros node Subscriber
										directly subscribe to joy message
*/

//include c language libraries
extern "C" {
#include "roboticscape.h"
}
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
#include <signal.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"

int c_port_cmd = 0,c_stdb_cmd=0;
std_msgs::Float64 c_port_duty, c_stdb_duty;
int running = 0;
int INIT_PULSE_WIDTH = 1000; //us
int WAKEUP_EN = 1;      // wakeup period enabled by default
double WAKEUP_S = 3.0;  // wakeup period in seconds
double WAKEUP_VAL = -0.1;// wakeup value
float MAX_PW_INCREMENT =200.0;// This is the hard limit for the thruster to prevent drawing too much current

int AUTO_FLAG=0;

void stdb_callback (const std_msgs::Float64::ConstPtr& msg)
{
	if(AUTO_FLAG==1)
	{
	c_stdb_duty.data = msg->data;
	c_stdb_cmd = (int)( c_stdb_duty.data*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
	}
}

void port_callback (const std_msgs::Float64::ConstPtr& msg)
{
	if(AUTO_FLAG==1)
	{
	c_port_duty.data = msg->data;
	c_port_cmd = (int)( c_port_duty.data*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
	}
}

void Motor_Callback (const sensor_msgs::Joy::ConstPtr& msg)
{
	//check for button press for motor driver relay
	if (msg->buttons[9] == 1)
  	{
 	   	
        	//wait for the esc
        	ROS_INFO("waking ESC up from idle for 3 seconds");
        	for(int i=0;i<=150;i++)
        	{
            		if(running==0) return;
            		if(rc_servo_send_esc_pulse_normalized(1,WAKEUP_VAL)==-1) return;
			if(rc_servo_send_esc_pulse_normalized(3,WAKEUP_VAL)==-1) return;
               		rc_usleep(1000000/50);
			rc_gpio_set_value(3,1,1); // turn on the relay after we setup the servo.
         	}
        	ROS_INFO("done with wakeup period");
  	}
	//Close the relay
  	if (msg->buttons[8] == 1)
  	{
    		rc_gpio_set_value(3, 1, 0);  // turn off the relays
  	}
	//only cmd PWM when it is postive
	if(msg->axes[1]>=0 & AUTO_FLAG==0)
	{
	//left stick
		c_port_duty.data = msg->axes[1];
		c_port_cmd = (int)(msg->axes[1]*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;	//set global pulse width varilable
	}
	if(msg->axes[3]>=0 &  AUTO_FLAG==0)
	{
	//right stick
		c_stdb_duty.data = msg->axes[3];
		c_stdb_cmd = (int)( msg->axes[3]*MAX_PW_INCREMENT) + INIT_PULSE_WIDTH;
	}

	
	//Set auto flag if "A" is pressed
	if(msg->buttons[1]==1)
	{
		AUTO_FLAG=1;
	}
	//disable auto flag if "B" is pressed
	if(msg->buttons[2]==1)
	{
		AUTO_FLAG=0;
	}
	
}

// interrupt handler to catch ctrl-c
void MySigintHandler(int sig)
{
	//shutting down
  	ROS_INFO("shutting down!");
	rc_servo_power_rail_en(0);
 	 rc_servo_cleanup();
  	//rc_gpio_cleanup(1, 25);
  	//rc_gpio_cleanup(1, 17);
	rc_gpio_cleanup(3, 1);
	rc_gpio_cleanup(3, 2);
	running = 0;
  	ros::shutdown();
}

int main(int argc, char** argv)
{
  	usleep(100000);  //there will be problems if there is no delay here

	//enable the IO which controls the motor driver relay
	if (rc_gpio_init(3, 1, GPIOHANDLE_REQUEST_OUTPUT) == -1)
	{
		printf("rc_gpio_init failed\n");
		return -1;
	}

	//setup servo
	if(rc_servo_init()) return -1;
        if(rc_servo_set_esc_range(RC_ESC_DEFAULT_MIN_US,RC_ESC_DEFAULT_MAX_US)) return -1;
        rc_servo_power_rail_en(0);

	  //if ctrl-c we shut down things
	signal(SIGINT, MySigintHandler);
  	running = 1;

	//ROS node stuff
  	ros::init(argc, argv, "auto_pwm");

  	ros::NodeHandle nh;

	//Remote control mode
	ros::Subscriber sub1 = nh.subscribe("/joy", 2, Motor_Callback);
	ros::Subscriber sub2 = nh.subscribe("/thrusters/stdb_cmd",2, stdb_callback);
	ros::Subscriber sub3 = nh.subscribe("/thrusters/port_cmd",2, port_callback);
	//ros::Publisher port_pwm_pub = nh.advertise<std_msgs::Float64>("/usv/port_motor/cmd", 2);
  	//ros::Publisher stdb_pwm_pub = nh.advertise<std_msgs::Float64>("/usv/stdb_motor/cmd", 2);


	ros::Rate loop_rate(50);  //esc need 50 Hz

	//START THE LOOP
  	while (ros::ok() & running)
  	{

	  	// send pwm adjust command
		rc_servo_send_pulse_us(1, c_port_cmd);//port side
		rc_servo_send_pulse_us(3, c_stdb_cmd);//starboard side

	    	ros::spinOnce();
    		loop_rate.sleep();

  	}
  	return 0;
}
