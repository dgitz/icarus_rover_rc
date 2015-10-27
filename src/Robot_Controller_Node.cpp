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


#include <sys/time.h>


using namespace std;

int DEBUG_MODE= 0;
int LOGGING_ENABLED = 0;
double dtime = 0.0;
string Operation_Mode = "";


::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
::icarus_rover_rc::ICARUS_GeneList GeneList;
int main(int argc, char **argv)
{
	
	
	
	int INITIALIZED = 1;

	ros::init(argc, argv, "Robot_Controller_Node");
	ros::NodeHandle nh("~");
	
	ICARUS_Diagnostic.System = ROVER;
	ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
	ICARUS_Diagnostic.Component = ROBOT_CONTROLLER_NODE;
	ICARUS_Diagnostic.Diagnostic_Type = NO_ERROR;
	ICARUS_Diagnostic.Level = INFORMATION;
	ICARUS_Diagnostic.Diagnostic_Message = INITIALIZING;
	
	//nh.getParam("target_count",target_count);
	nh.getParam("DEBUG_MODE",DEBUG_MODE);
	nh.getParam("LOGGING_ENABLED",LOGGING_ENABLED);
	nh.getParam("Operation_Mode",Operation_Mode); //Should be: SIM, LIVE
	if(DEBUG_MODE == 0)
	{
		
	}
	else if(DEBUG_MODE == 1)
	{
		
	}
	time_t rawtime;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	char filename[80];
	strcpy(filename,ctime(&rawtime));
	strcat(filename,"RobotControllerNode.csv");
	ofstream out;
	if(LOGGING_ENABLED == 1) 
	{
		ofstream out(filename);
		//out << "Time,GPS Valid,Armed,Gear,Drive_Command,Steer_Command,current_speed,current_latitude,current_longitude,current_heading,Northing,Easting" << endl;
	}
	ros::Publisher Pub_ICARUS_Robot_Controller_Node_Diagnostic;
	ros::Publisher Pub_ICARUS_GeneList;
	if(Operation_Mode == "LIVE")
	{
	}
	else if(Operation_Mode == "SIM")
	{
		Pub_ICARUS_Robot_Controller_Node_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Robot_Controller_Node_Diagnostic",1000);

		Pub_ICARUS_GeneList = nh.advertise<icarus_rover_rc::ICARUS_GeneList>("ICARUS_GeneList",1000);

	}
	Pub_ICARUS_Robot_Controller_Node_Diagnostic.publish(ICARUS_Diagnostic);

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
				int GeneCount = 10;
				GeneList.GeneName.resize(GeneCount);
				GeneList.GeneValue.resize(GeneCount);
				Pub_ICARUS_GeneList.publish(GeneList);
			}
			catch(const std::exception& ex)
			{
				ROS_INFO("ERROR:%s",ex.what());
			}
		}
		

		
		dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
	    Pub_ICARUS_Robot_Controller_Node_Diagnostic.publish(ICARUS_Diagnostic);
		
	}
	if(LOGGING_ENABLED == 1)
	{
		out.close();
	}
	
}
