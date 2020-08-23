/* 
Author Jianguang Shi, sjg@hdu.edu.cn
This driver is created to read GPS NMEA from serial port and publish it into /gps_data through a GPSFix.msg.
 */
 
#pragma once
#include "ros/ros.h"
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
#include "gps_common/GPSFix.h"

#define DEFAULT_PORT "/dev/ttyACM0"    // Default port name
#define DEFAULT_BAUD 9600              // Default baudate
#define DEFAULT_FRAME_ID "gps"
 
#ifndef GPSINT_H
#define GPSINT_H
 
#define PI (3.14159265359)
#define GPSBUFSIZE  256       // GPS buffer size
 
/**
 * GPSINT Class.
 */
 
 
	class GPS{
public:
    /**
     * Constructor.
     * @param gps - Serial port pins attached to the gps
     */ 
	GPS(ros::NodeHandle nh);
    int nmea_validate(char *nmeastr);               //runs the checksum calculation on the GPS NMEA string
    void parse_gps(char *GPSstrParse);         //uses scanf to parse NMEA string into variables
    void read_serial(void);              //fills temprpary buffer for processing
    float nmea_to_dec(float deg_coord, char nsew);  //convert nmea format to decimal format
    float calc_course_to(float pointLat, float pontLong);
    double calc_dist_to_mi(float pointLat, float pontLong);
    double calc_dist_to_ft(float pointLat, float pontLong);
    double calc_dist_to_km(float pointLat, float pontLong);
    double calc_dist_to_m(float pointLat, float pontLong);
    std::string port_name,frame_id;
	int baud;
    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;
    
    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;    
    float utc_time;
    char ns, ew,ww,nn;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;
    
    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;
    
    // GLL
    char gll_status;
    
    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;
private:
    serial::Serial *ser;
	ros::Publisher gps_pub;
};
 
#endif /* GPSINT_H */