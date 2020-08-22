/* gps_driver.cpp
Author Jianguang Shi, sjg@hdu.edu.cn
This driver is created to read GPS NMEA from serial port and publish it into /gps_data through a GPSFix.msg.
The track is in  enu frame with the unit degee, true north.
The unit of speed is m/s.
 */
#include <gps_driver.h> 
 
 GPS::GPS(ros::NodeHandle nh)
{
	ros::NodeHandle nh_priv("~");
	nh_priv.param<std::string>("port_name", port_name, DEFAULT_PORT);
	nh_priv.param("baud", baud, DEFAULT_BAUD);
	nh_priv.param<std::string>("frame_id", frame_id, DEFAULT_FRAME_ID);	
    ser = new serial::Serial(port_name,baud, serial::Timeout::simpleTimeout(1000));
	ser->flush();
	ROS_INFO("GPS");
	gps_pub = nh.advertise<gps_common::GPSFix>("sensor/gps_data",2);
}

 void GPS::read_serial()
 {
	 char _data[256]={0};
	 std::string s_data;
	 std::size_t sz_t = 256;
	 	if(ser->available())
	{
		s_data = ser->readline(sz_t,"\n");
		strcpy(_data,s_data.c_str());
	}
	if(nmea_validate(_data)){
        parse_gps(_data);
    }
 }

 
int GPS::nmea_validate(char *nmeastr){
    char check[3];
    char checkcalcstr[3];
    int i;
    int calculated_check;
 
    i=0;
    calculated_check=0;
 
    // check to ensure that the string starts with a $
    if(nmeastr[i] == '$')
        i++;
    else
        return 0;
 
    //No NULL reached, 75 char largest possible NMEA message, no '*' reached
    while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)){
        calculated_check ^= nmeastr[i];// calculate the checksum
        i++;
    }
 
    if(i >= 75){
        return 0;// the string was too long so return an error
    }
 
    if (nmeastr[i] == '*'){
        check[0] = nmeastr[i+1];    //put hex chars in check string
        check[1] = nmeastr[i+2];
        check[2] = 0;
    }
    else
        return 0;// no checksum separator found therefor invalid
 
    sprintf(checkcalcstr,"%02X",calculated_check);
    return((checkcalcstr[0] == check[0])
        && (checkcalcstr[1] == check[1])) ? 1 : 0 ;
} 
 
void GPS::parse_gps(char *GPSstrParse){
    //check if $GPGGA string
/*     if(!strncmp(GPSstrParse, "$GPGGA", 6)){
        if (sscanf(GPSstrParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &utc_time, &nmea_latitude, &ns, &nmea_longitude, &ew, &lock, &satelites, &hdop, &msl_altitude, &msl_units) >= 1) {
//            printf("%s", GPSstrParse);
        }
        else{
 //           printf("BAD parse %s", GPSstrParse);    
        }
    } */
    // Check if $GPRMC string
    if (!strncmp(GPSstrParse, "$GPRMC", 6)){  
        sscanf(GPSstrParse, "$GPRMC,%f,%c,%f,%c,%f,%c,%f,%f,%d", &utc_time, &ns, &nmea_latitude, &ew, &nmea_longitude, &ww, &speed_k, &course_d, &date);
        printf("%s", GPSstrParse);
		nmea_longitude=nmea_to_dec(nmea_longitude,ew);
		nmea_latitude=nmea_to_dec(nmea_latitude,ww);
		course_d=90-course_d;
		if(course_d>180)
		{
			course_d=course_d-360;
		}
		if(course_d<-180)
		{
			course_d=course_d+360;
		}
		speed_k=0.5144444*speed_k;
		ROS_INFO("lon:%f, lat:%f, track:%f, speed:%f",nmea_longitude,nmea_latitude,course_d,speed_k);
		gps_common::GPSFix pub_data;
		pub_data.header.frame_id=frame_id;
		pub_data.header.stamp= ros::Time::now();
		pub_data.longitude = nmea_longitude;
		pub_data.latitude = nmea_latitude;
		pub_data.track = course_d;
		pub_data.speed = speed_k;
		pub_data.time = utc_time;
		gps_pub.publish(pub_data);
		return;		
		//printf("sssssssssss:%f", utc_time);
    }
    // GLL - Geographic Position-Lat/Lon
/*     else if (!strncmp(GPSstrParse, "$GPGLL", 6)){
        if(sscanf(GPSstrParse, "$GPGLL,%f,%c,%f,%c,%f,%c", &nmea_latitude, &ns, &nmea_longitude, &ew, &utc_time, &gll_status) >= 1) {
//            printf("%s", GPSstrParse);
            return;
        }
        else{
 //           printf("BAD parse %s", GPSstrParse);    
        }        
    }
    // VTG-Course Over Ground and Ground Speed
    else if (!strncmp(GPSstrParse, "$GPVTG", 6)){
        if(sscanf(GPSstrParse, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c", &course_t, &course_t_unit, &course_m, &course_m_unit, &speed_k, &speed_k_unit, &speed_km, &speed_km_unit) >= 1) {
//            printf("%s", GPSstrParse);
            return;
        }
        else{
//            printf("BAD parse %s", GPSstrParse);    
        }        
    } */
}//parseGPSstring()
 
     
float GPS::nmea_to_dec(float deg_coord, char nsew) {
    int degree = (int)(deg_coord/100);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}
 
// NAVIGATION FUNCTIONS ////////////////////////////////////////////////////////////
float GPS::calc_course_to(float pointLat, float pontLong) {
    const double d2r = PI / 180.0;
    const double r2d = 180.0 / PI;
    double dlat = abs(pointLat - dec_latitude) * d2r;
    double dlong = abs(pontLong - dec_longitude) * d2r;
    double y = sin(dlong) * cos(pointLat * d2r);
    double x = cos(dec_latitude*d2r)*sin(pointLat*d2r) - sin(dec_latitude*d2r)*cos(pointLat*d2r)*cos(dlong);
    return atan2(y,x)*r2d;
}    
 
/*
var y = Math.sin(dLon) * Math.cos(lat2);
var x = Math.cos(lat1)*Math.sin(lat2) -
        Math.sin(lat1)*Math.cos(lat2)*Math.cos(dLon);
var brng = Math.atan2(y, x).toDeg();
*/
 
/*
            The Haversine formula according to Dr. Math.
            http://mathforum.org/library/drmath/view/51879.html
                
            dlon = lon2 - lon1
            dlat = lat2 - lat1
            a = (sin(dlat/2))^2 + cos(lat1) * cos(lat2) * (sin(dlon/2))^2
            c = 2 * atan2(sqrt(a), sqrt(1-a)) 
            d = R * c
                
            Where
                * dlon is the change in longitude
                * dlat is the change in latitude
                * c is the great circle distance in Radians.
                * R is the radius of a spherical Earth.
                * The locations of the two points in 
                    spherical coordinates (longitude and 
                    latitude) are lon1,lat1 and lon2, lat2.
*/
double GPS::calc_dist_to_mi(float pointLat, float pontLong) {
    const double d2r = PI / 180.0;
    double dlat = pointLat - dec_latitude;
    double dlong = pontLong - dec_longitude;
    double a = pow(sin(dlat/2.0),2.0) + cos(dec_latitude*d2r) * cos(pointLat*d2r) * pow(sin(dlong/2.0),2.0);
    double c = 2.0 * asin(sqrt(abs(a)));
    double d = 63.765 * c;
    
    return d;
}
 
double GPS::calc_dist_to_ft(float pointLat, float pontLong) {
    return calc_dist_to_mi(pointLat, pontLong)*5280.0;
}
 
double GPS::calc_dist_to_km(float pointLat, float pontLong) {
    return calc_dist_to_mi(pointLat, pontLong)*1.609344;
}
 
double GPS::calc_dist_to_m(float pointLat, float pontLong) {
    return calc_dist_to_mi(pointLat, pontLong)*1609.344;
}
