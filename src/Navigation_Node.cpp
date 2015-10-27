//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <nav_msgs/Path.h>
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
#include "icarus_rover_rc/RC.h"
#include "icarus_rover_rc/VFR_HUD.h"
#include "icarus_rover_rc/ICARUS_Diagnostic.h"
#include "icarus_rover_rc/ICARUS_GeneList.h"
#include "icarus_rover_rc/State.h"
#define PI 3.14159265359
#define STEER_MAX_LEFT 1850 //MECHANICAL MAX: 1980
#define STEER_MAX_RIGHT 1300 //MECHANICAL MAX: 1160
#define STEER_CENTER 1620
#define DRIVE_MAX_FORWARD 1650 //MECHANICAL MAX: 2000
#define DRIVE_MAX_REVERSE 1350  //MECHANICAL MAX: 1400
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
int Joystick_Steer_Axis = 0;
int Joystick_Throttle_Axis = 3;
int Joystick_Disarm_Button = 14;
int Joystick_Arm_Button = 12;
double Desired_Steering_Percentage = 0.0;
double Desired_Throttle = 0.0;
//Position Variables
float current_latitude = 0.0;
float current_longitude = 0.0;
double current_Northing_m = 0.0;
double current_Easting_m = 0.0;
double origin_Northing_m = 0.0;
double origin_Easting_m = 0.0;
double temp_Northing = 0.0;
double temp_Easting = 0.0;
float current_speed = 0.0;
int current_speed_command = 0;
float current_Heading_deg = 0.0;
//Other Variables

int DEBUG_MODE= 0;
int LOGGING_ENABLED = 0;
double dtime = 0.0;
string Operation_Mode = "";
geometry_msgs::Pose2D GoalPose;
sensor_msgs::Joy RoverCommand;

geometry_msgs::Pose2D TargetPose;
int Simulation_Initialized = 0;
int Goal_Reached = 0;


::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
void GlobalPath_Callback(const nav_msgs::Path::ConstPtr& msg)
{
	int PathSize = msg->poses.size();
	float Path_LookAhead = .25;
	int path_index = Path_LookAhead*PathSize;
	if(path_index < 0){path_index = 0;}
	else if(path_index > PathSize) { path_index = 0; }
	/*if(Path_LookAhead > PathSize) 
	{	
		TargetPose.x = msg->poses[0].pose.position.x;
		TargetPose.y = msg->poses[0].pose.position.y;
	}
	else
	{*/
		TargetPose.x = msg->poses[path_index].pose.position.x;
		TargetPose.y = msg->poses[path_index].pose.position.y;
	//}
	/*for(int i = 0; i < PathSize;i++)
	{
		printf("P[%d] N: %f E: %f\r\n",i,msg->poses[i].pose.position.y,msg->poses[i].pose.position.x);
	}*/
	//TargetPose.theta = current_Heading_deg
	double dx = TargetPose.x-current_Easting_m;
	double dy = TargetPose.y-current_Northing_m;
	double Target_Bearing_deg = (atan2(dx,dy)*180/PI);
	//printf("N: %f E: %f Heading: %f Bearing: %f Tx %f Ty: %f dx: %f dy: %f\r\n",current_Northing_m,current_Easting_m,current_Heading_deg,Target_Bearing_deg,TargetPose.x,TargetPose.y,dx,dy);
	//printf("Tx:%f,%f\r\n",TargetPose.x,msg->poses[Path_LookAhead].pose.position.x);
	if(Goal_Reached == 1)
	{
		Desired_Throttle = 0.0;
	}
	else
	{
		Desired_Throttle = 0.5;
	}
	Desired_Steering_Percentage = (Target_Bearing_deg-current_Heading_deg)/180.0;
	
}

void ICARUS_Rover_Goal_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	Simulation_Initialized = 1;
	GoalPose.x = msg->x;
	GoalPose.y = msg->y;
	GoalPose.theta = msg->theta;
	//printf("Got a Goal x: %f y: %f theta: %f\r\n",msg->x,msg->y,msg->theta);
	
}

void GeneList_Callback(const icarus_rover_rc::ICARUS_GeneList::ConstPtr& msg)
{
	//printf("Nav Node: Got a new Gene List!\r\n");
}
void ICARUS_Sonar_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//printf("Got a Scan!\r\n");
}
void ICARUS_Rover_VFRHUD_Callback(const icarus_rover_rc::VFR_HUD::ConstPtr& msg)
{
	if(DEBUG_MODE == 0)
	{
		current_Heading_deg = (float)msg->heading;
		current_speed = msg->groundspeed;
	}
}
void ICARUS_SimRover_Pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	//printf("x: %f y: %f head: %f\r\n",msg->x,msg->y,msg->theta);
	current_Easting_m = msg->x;
	current_Northing_m = msg->y;
	current_Heading_deg = msg->theta*180.0/PI;
	double dx = fabs(current_Easting_m - GoalPose.x);
	double dy = fabs(current_Northing_m - GoalPose.y);
	//printf("Goal E: %f E: %f Goal N: %f N: %f dx: %f dy: %f\r\n",GoalPose.x,current_Easting_m,GoalPose.y,current_Northing_m,dx,dy);
	if((dx < 0.5) and (dy < 0.5) and (Simulation_Initialized == 1))
	{
		printf("GOAL REACHED!\r\n");
		Goal_Reached = 1;
	}
	
}

int main(int argc, char **argv)
{
	
	
	
	int INITIALIZED = 1;

	ros::init(argc, argv, "Naviation_Node");
	ros::NodeHandle nh("~");
	
	ICARUS_Diagnostic.System = ROVER;
	ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
	ICARUS_Diagnostic.Component = NAVIGATION_NODE;
	ICARUS_Diagnostic.Diagnostic_Type = NO_ERROR;
	ICARUS_Diagnostic.Level = INFORMATION;
	ICARUS_Diagnostic.Diagnostic_Message = INITIALIZING;
	
	//nh.getParam("target_count",target_count);
	nh.getParam("DEBUG_MODE",DEBUG_MODE);
	nh.getParam("LOGGING_ENABLED",LOGGING_ENABLED);
	nh.getParam("Operation_Mode",Operation_Mode); //Should be: SIM, LIVE
	if(DEBUG_MODE == 0)
	{
		current_Easting_m = origin_Easting_m;
		current_Northing_m = origin_Northing_m;
	}
	else if(DEBUG_MODE == 1)
	{
		current_Easting_m = 0.0;
		current_Northing_m = 0.0;
	}
	time_t rawtime;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	char filename[80];
	strcpy(filename,ctime(&rawtime));
	strcat(filename,"NavigationNode.csv");
	ofstream out;
	if(LOGGING_ENABLED == 1) 
	{
		ofstream out(filename);
		//out << "Time,GPS Valid,Armed,Gear,Drive_Command,Steer_Command,current_speed,current_latitude,current_longitude,current_heading,Northing,Easting" << endl;
	}
	ros::Publisher Pub_ICARUS_Navigation_Node_Diagnostic;
	ros::Subscriber Sub_Rover_Pose;
	ros::Subscriber Sub_Rover_Goal;
	ros::Publisher Pub_Rover_Command;
	ros::Subscriber Sub_Rover_GlobalPath;
	ros::Subscriber Sub_Sonar_Scan;
	ros::Subscriber Sub_GeneList;
	RoverCommand.axes.resize(8);
	RoverCommand.buttons.resize(8);
	////Pub_Rover_GlobalPath = nh.advertise<nav_msgs::Path>("ICARUS_Rover_GlobalPath",1000);
	if(Operation_Mode == "LIVE")
	{
		/*Pub_ICARUS_Navigation_Node_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Motion_Controller_Diagnostic",1000);
		Sub_Rover_Control = nh.subscribe<sensor_msgs::Joy>("/joy",1000,ICARUS_Rover_Control_Callback);
		GPS_State = nh.subscribe<sensor_msgs::NavSatFix>("/Mavlink_Node/gps",1000,ICARUS_Rover_GPS_Callback); 
		VFRHUD_State = nh.subscribe<icarus_rover_rc::VFR_HUD>("/Mavlink_Node/vfr_hud",1000,ICARUS_Rover_VFRHUD_Callback);
		Pub_Rover_RC = nh.advertise<icarus_rover_rc::RC>("send_rc",1000);
		Pub_Rover_Pose = nh.advertise<geometry_msgs::Pose2D>("/Motion_Controller_Node/ICARUS_Rover_Pose",1000);
		Sub_Rover_GlobalPath = nh.subscribe<nav_msgs::Path>("/Mapping_Node/ICARUS_Rover_GlobalPath",1000,GlobalPath_Callback);*/
	}
	else if(Operation_Mode == "SIM")
	{
		Pub_ICARUS_Navigation_Node_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Motion_Controller_Diagnostic",1000);
		Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Pose",1000,ICARUS_SimRover_Pose_Callback);
		Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Sonic_Controller_Node/ICARUS_Sonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
		Sub_Rover_Goal = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Goal",1000,ICARUS_Rover_Goal_Callback);
		Pub_Rover_Command = nh.advertise<sensor_msgs::Joy>("ICARUS_Rover_Command",1000);
		Sub_Rover_GlobalPath = nh.subscribe<nav_msgs::Path>("/Mapping_Node/ICARUS_Rover_GlobalPath",1000,GlobalPath_Callback);
		Sub_GeneList = nh.subscribe<icarus_rover_rc::ICARUS_GeneList>("/Robot_Controller_Node/ICARUS_GeneList",1000,GeneList_Callback);
	}
	Pub_ICARUS_Navigation_Node_Diagnostic.publish(ICARUS_Diagnostic);

	ros::Rate loop_rate(50);
	std::clock_t    start;
	
	int direction = 1;
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	while( ros::ok() && INITIALIZED)
	{
		
		start = std::clock();
	  	ros::spinOnce();
		current_time = ros::Time::now();
	  	loop_rate.sleep();
		if(Operation_Mode == "LIVE")
		{
			try
			{
				
			}		
			catch(const std::exception& ex)
			{
				ROS_INFO("ERROR:%s",ex.what());
		  
				
			}
		
		}
		else if(Operation_Mode == "SIM")
		{
			try
			{
				RoverCommand.header.stamp = ros::Time::now();
				RoverCommand.axes[Joystick_Steer_Axis] = Desired_Steering_Percentage;
				RoverCommand.axes[Joystick_Throttle_Axis] = Desired_Throttle;
				Pub_Rover_Command.publish(RoverCommand);
			}
			catch(const std::exception& ex)
			{
				ROS_INFO("ERROR:%s",ex.what());
			}
		}
		

		
		dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
	    Pub_ICARUS_Navigation_Node_Diagnostic.publish(ICARUS_Diagnostic);
		
	}
	if(LOGGING_ENABLED == 1)
	{
		out.close();
	}
	
}
