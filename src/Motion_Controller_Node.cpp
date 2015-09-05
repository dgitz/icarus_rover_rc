//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <dirent.h>
//#include <gps_common/conversions.h>
#include "icarus_rover_rc/Definitions.h"
#include "icarus_rover_rc/conversions.h"
#include "icarus_rover_rc/ICARUS_Probe_Status.h"
#include "icarus_rover_rc/ICARUS_Probe_Command.h"
#include "icarus_rover_rc/RC.h"
#include "icarus_rover_rc/VFR_HUD.h"
//#include "icarus_rover_rc/ICARUS_Diagnostic.h"

#include "icarus_rover_rc/State.h"
#define PI 3.14159265359
#define STEER_MAX_LEFT 1850 //MECHANICAL MAX: 1980
#define STEER_MAX_RIGHT 1300 //MECHANICAL MAX: 1160
#define STEER_CENTER 1620
#define DRIVE_MAX_FORWARD 1600 //MECHANICAL MAX: 2000
#define DRIVE_MAX_REVERSE 1400  //MECHANICAL MAX: 1400
#define DRIVE_NEUTRAL 1500

#define STEER_CHANNEL 0 //Motion Controller Steer PWM Pin
#define DRIVE_CHANNEL 2 //Motion Controller Drive PWM Pin

#include <sys/time.h>
void updatePose(double latitude,double longitude);

using namespace std;


int Gps_Valid = 0; 

//Motion Variables
int Steer_Command = STEER_CENTER;
int Drive_Command = DRIVE_NEUTRAL;
int armed_state = DISARMED;
int joystick_timeout_counter = 0;
int current_gear = GEAR_PARK;

//Position Variables
float current_latitude = 0.0;
float current_longitude = 0.0;
double current_Northing = 0.0;
double current_Easting = 0.0;
float current_speed = 0.0;
int current_speed_command = 0;
int current_heading = 0;

//Other Variables

int DEBUG_MODE;
double dtime = 0.0;






void ICARUS_Rover_GPS_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	if(msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
	{ 
		Gps_Valid = 0; 
		printf("GPS Status: NO FIX.\r\n");
	}
	else
	{
		Gps_Valid = 1;
		//current_latitude = msg->latitude;
		//current_longitude = msg->longitude;
		//updatePose(msg->latitude,msg->longitude);
		
		//printf("GPS Status: FIX AVAILABLE.\r\n");
		std::string zone;
		current_latitude = msg->latitude;
		current_longitude = msg->longitude;
		LLtoUTM(msg->latitude,msg->longitude,current_Northing,current_Easting,zone);
	}
	
}
void updatePose(double latitude,double longitude)
{
	int32_t Equatorial_Radius = 6378137;
	double Eccentricity = .081819;
	
	int longitude_zone = 31+(int)(longitude/6);
	double delta_longitude_rad = (double)(longitude-longitude_zone)*PI/180.0;
	double latitude_rad = (double)(latitude*PI/180.0);
	double longitude_rad = (double)(longitude*PI/180.0);
	double temp1 = (Eccentricity*sin(latitude_rad))*(Eccentricity*sin(latitude_rad));
	double rcurv_1 = Equatorial_Radius*(1.0-Eccentricity*Eccentricity)/pow((1-temp1),1.5);
	double rcurv_2 = Equatorial_Radius/pow((1.0-temp1),0.5);

	
}
void ICARUS_Rover_VFRHUD_Callback(const icarus_rover_rc::VFR_HUD::ConstPtr& msg)
{
	current_heading = msg->heading;
	current_speed = msg->groundspeed;
}
void ICARUS_Rover_Control_Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	
	//Assume Joy values are float, -1.0 to 1.0
  //Steer_Command = (int)(joy->axes[0]+0.5)*255.0;
  //Drive_Command = (int)(joy->axes[1]+0.5)*255.0; 
  //printf("X: %f Y: %f",joy->axes[0],joy->axes[3]);
  int steer_axis = 0;
  int drive_axis = 3;
  int disarm_button = 14;
  int arm_button = 12;
  float deadband = 0.05;
  joystick_timeout_counter = 0; //Reset Joystick Timeout Counter
  float steer_axis_value = joy->axes[steer_axis];
  float drive_axis_value = joy->axes[drive_axis];
  if(joy->buttons[disarm_button] == 1) { armed_state = DISARMED; }
  else if(joy->buttons[arm_button] == 1) { armed_state = ARMED; }
  if(armed_state == ARMED)
  {
	  if(steer_axis_value > deadband) // Turning Left
	  {
		Steer_Command = STEER_CENTER + steer_axis_value * (STEER_MAX_LEFT - STEER_CENTER);
	  }
	  else if(steer_axis_value < (-1.0*deadband)) //Turning Right
	  {
		Steer_Command = STEER_CENTER + steer_axis_value * (STEER_CENTER - STEER_MAX_RIGHT);
	  }
	  else
	  {
		Steer_Command = STEER_CENTER;
	  }
	  if(drive_axis_value > deadband) //Moving Forward
	  {
		current_gear = GEAR_FORWARD;
		Drive_Command = DRIVE_NEUTRAL + drive_axis_value * (DRIVE_MAX_FORWARD - DRIVE_NEUTRAL);
	  }
	  else if(drive_axis_value < (-1.0*deadband)) //Moving Reverse
	  {
		Drive_Command = DRIVE_NEUTRAL + drive_axis_value * (DRIVE_NEUTRAL - DRIVE_MAX_REVERSE);
		current_gear = GEAR_REVERSE;
	  }
	  else
	  {
		Drive_Command = DRIVE_NEUTRAL;
		current_gear = GEAR_PARK;
	  }
  }
  
  
  
}
int main(int argc, char **argv)
{
	int INITIALIZED = 1;
	ros::init(argc, argv, "Motion_Controller");
	ros::NodeHandle nh("~");
	//nh.getParam("target_count",target_count);
	nh.getParam("DEBUG_MODE",DEBUG_MODE);
	time_t rawtime;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	char filename[80];
	strcpy(filename,ctime(&rawtime));
	strcat(filename,".txt");
	ofstream out(filename);
	out << "Time,GPS Valid,Armed,Gear,Drive_Command,Steer_Command,current_speed,current_latitude,current_longitude,current_heading,Northing,Easting" << endl;
	//ros::Publisher Pub_ICARUS_Motion_Controller_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Motion_Controller_Diagnostic",1000);
	ros::Subscriber Sub_Rover_Control = nh.subscribe<sensor_msgs::Joy>("/joy",1000,ICARUS_Rover_Control_Callback);
	ros::Subscriber GPS_State = nh.subscribe<sensor_msgs::NavSatFix>("/Mavlink_Node/gps",1000,ICARUS_Rover_GPS_Callback); 
	ros::Subscriber VFRHUD_State = nh.subscribe<icarus_rover_rc::VFR_HUD>("/Mavlink_Node/vfr_hud",1000,ICARUS_Rover_VFRHUD_Callback);
	ros::Publisher Pub_Rover_RC = nh.advertise<icarus_rover_rc::RC>("send_rc",1000);
	//ros::Publisher Pub_ICARUS_Rover_Pose = nh.advertise<geometry_msgs::Pose2D>("ICARUS_Rover_Pose",1000);

	ros::Rate loop_rate(50);
	std::clock_t    start;
	::icarus_rover_rc::ICARUS_Probe_Status Probe_Status;
	/*::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
	ICARUS_Diagnostic.header.frame_id = "ICARUS_Motion_Controller_Diagnostic";
	ICARUS_Diagnostic.System = ROVER;
	ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
	ICARUS_Diagnostic.Component = MOTION_CONTROLLER_NODE;
	*/
	::icarus_rover_rc::RC RC_Command;
	int direction = 1;
	while( ros::ok() && INITIALIZED)
	{
    
		start = std::clock();
	  	ros::spinOnce();
	  	loop_rate.sleep();
		joystick_timeout_counter++;  //Increment Joystick Timeout Counter
		if(joystick_timeout_counter > 50) 
		{ 
			printf("Joystick Callback Timeout\r\n");// }
			armed_state = DISARMED; 
		}
	  	if(armed_state == ARMED)
	  	{
			if(DEBUG_MODE >= INFORMATION) { printf("Armed State: ARMED\r\n"); }
		}
		else if(armed_state == DISARMED)
		{
			if(DEBUG_MODE >= INFORMATION) { printf("Armed State: DISARMED\r\n"); }
			Steer_Command = STEER_CENTER;
			Drive_Command = DRIVE_NEUTRAL;

		}
		else
		{
			printf("Armed State: UNKNOWN!!!\r\n");
		}
	  	try
		{
			

			//for(int i = 0; i < 7; i++) { RC_Command.channel[i] = Steer_Command; }
			RC_Command.channel[STEER_CHANNEL] = Steer_Command;
			RC_Command.channel[DRIVE_CHANNEL] = Drive_Command;
			
			if(Gps_Valid == 1)
			{
				//cout << "Gear: " << current_gear << " Thr: " << Drive_Command << " Sp: " << current_speed << " GPS: " << Gps_Valid << " Lat: " << current_latitude << " Long: " << current_longitude << " Head: " << current_heading << endl;
				//
				struct timeval mytime;
				gettimeofday(&mytime,NULL);
				double cur_time = (double)(mytime.tv_sec + mytime.tv_usec/1000000.0);
				
				
				out << setprecision(16) << cur_time << "," << Gps_Valid << "," << armed_state << "," << current_gear << "," << Drive_Command << "," << Steer_Command << "," << current_speed << "," << current_latitude << "," << current_longitude << "," << current_heading << "," << current_Northing << "," << current_Easting <<  endl;
				printf("T: %f Armed: %d Gear: %d Thr: %d St: %d Sp: %f GPS: %d Lat: %4.12f Long: %4.12f Head: %d N: %f E: %f\r\n",cur_time,armed_state,current_gear,Drive_Command,Steer_Command,current_speed,Gps_Valid,current_latitude,current_longitude,current_heading,current_Northing,current_Easting);
			}
			//for(int i = 0; i < 8; i++) { RC_Command.channel[i] = Steer_Command; }
			
			Pub_Rover_RC.publish(RC_Command);
			//if (DEBUG_MODE == 1) { printf("$NAV,%d,%d,*\r\n",(int)Steer_Command,(int)Drive_Command);}
			//printf();
			if(1)
			{
				
				


				/*ICARUS_Diagnostic.Diagnostic_Type = COMMUNICATIONS;
				ICARUS_Diagnostic.Level = DEBUG;
				ICARUS_Diagnostic.Diagnostic_Message = NO_ERROR;
				*/
				//ICARUS_Diagnostic.Description =  "";
				if (DEBUG_MODE == 1) 
				{
					/*if(Gps_Valid == 0) { cout << "GPS: NO FIX." << endl;}
					else if(Gps_Valid == 1) { cout << "GPS: NAV READY." << endl; }*/
				}
				//Pub_ICARUS_Rover_Pose.publish(Rover_Pose);
			}
			
			
			else
			{
				/*ICARUS_Diagnostic.header.stamp = ros::Time::now();
				ICARUS_Diagnostic.Diagnostic_Type = COMMUNICATIONS;
				ICARUS_Diagnostic.Level = CAUTION;
				ICARUS_Diagnostic.Diagnostic_Message = DROPPING_PACKETS;
				Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);*/
			}
			dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
     


			  //ICARUS Diagnostics Publisher
			  /*ICARUS_Diagnostic.header.stamp = ros::Time::now();
			  Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);*/
		}
		catch(const std::exception& ex)
		{
			ROS_INFO("ERROR:%s",ex.what());
      
			//ICARUS Diagnostics Publisher
			/*ICARUS_Diagnostic.header.stamp = ros::Time::now();
			ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
			ICARUS_Diagnostic.Level = FATAL;
			ICARUS_Diagnostic.Diagnostic_Message = GENERAL_ERROR;
			ICARUS_Diagnostic.Description = ex.what();
			Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);*/
		}
	}
	out.close();
	/*
	ICARUS_Diagnostic.header.stamp = ros::Time::now();
	ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
	ICARUS_Diagnostic.Level = SEVERE;
	ICARUS_Diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
	ICARUS_Diagnostic.Description = "Node Closed.";
	Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
  */
}
