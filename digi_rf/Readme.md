This file include all the msg IDs and types for the RF.

MSG ID|MSG example
-----------Robot 1---------------------------
00| $FOO,10,20,30,40*42\r\n   #foo message
01| $IMU,frame,time,ori.x,ori.y,ori.z,orientation.w,ang_v.x,ang_v.y,ang_v.z,l_v.x,l_v.y,l_v.z*cs\r\n
02| $GPS,frame,sys_time,lat,lon,cog,sog,roll,pitch,UTC_time*cs\r\n
03| $ODO,frame, child_frame, time, pose.x,pose.y,pose.z,ori.x,ori.y,ori.z,ori.w*cs\r\n
04| $CTR,1-AUTO 0-Manual,STDB_DC,PORT_DC,c_heading, c_wptx, c_wpt_y*cs\r\n
05| $SYS,time,V_main,I_main,V_payload,I_payload*cs\r\n
06| reserved
07| $JOY,time,leftstick(ls), rightstick(rs), start, back, A,B,X,Y*cs\r\n
08| $reserved
09| $reserved
-----------Robot 2---------------------------
10|
11|
12|
13|
.....


Steps to add your own nmea message for other ros messages
1.
2.
3.
4.
5.
