//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <dirent.h>
#include <iomanip>
#include <ctime>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include "icarus_rover_rc/Definitions.h"
#include "icarus_rover_rc/ICARUS_Diagnostic.h"

#define LINE1 254
#define BACKLIGHT_ON 157
#define BACKLIGHT_OFF 128
using namespace std;

//Communication Variables
int token_index = 0;
string DIAG_device = "";
int Baud_Rate = -1;
int Output_Level = 1;

double dtime = 0.0;

void Diagnostic_Callback(const icarus_rover_rc::ICARUS_Diagnostic::ConstPtr& msg)
{
	
	if (msg->Level >= Output_Level)
	{
		switch(msg->System)
		{
			case ROVER:
				printf("System: Rover ");
				break;
			case GROUND_STATION:
				printf("System: Ground Station ");
				break;
			case REMOTE_CONTROL:
				printf("System: Remote Control ");
				break;
			default:
				break;
		}
		switch(msg->SubSystem)
		{
			case ENTIRE_SYSTEM:
				printf("Subystem: Entire System ");
				break;
			case ROBOT_CONTROLLER:
				printf("Subsystem: Robot Controller ");
				break;
			case MOTION_CONTROLLER:
				printf("Subsystem: Motion Controller ");
				break;
			case SONIC_CONTROLLER:
				printf("Subsystem: Sonic Controller ");
				break;
			default:
				break;
		} 
		switch(msg->Component)
		{
			case ENTIRE_SUBSYSTEM:
				printf("Component: Entire Subsystem ");
				break;
			case DIAGNOSTIC_NODE:
				printf("Component: Diagnostic Node ");
				break;
			case NAVIGATION_NODE:
				printf("Component: Navigation Node ");
				break;
			case MOTION_CONTROLLER_NODE:
				printf("Component: Motion Controller Node ");
				break;
			case SONIC_CONTROLLER_NODE:
				printf("Component: Sonic Controller Node ");
				break;
			case EVOLUTION_NODE:
				printf("Component: Evolution Node ");
				break;
			case TARGETING_NODE:
				printf("Component: Targeting Node ");
				break;
			case MAPPING_NODE:
				printf("Component: Mapping Node ");
				break;
			default:
				break;
		}
		switch(msg->Diagnostic_Type)
		{
			case NO_ERROR:
				printf("Type: No Error ");
				break;
			case ELECTRICAL:
				printf("Type: Electrical ");
				break;
			case SOFTWARE:
				printf("Type: Software ");
				break;
			case COMMUNICATIONS:
				printf("Type: Communications ");
				break;
			case SENSORS:
				printf("Type: Sensors ");
				break;
			case ACTUATORS:
				printf("Type: Actuators ");
				break;
			case DATA_STORAGE:
				printf("Type: Data Storage ");
				break;
			case GENERAL_ERROR:
				printf("Type: General Error ");
				break;
			default:
				break;
		}
		
		switch(msg->Level)
		{
			case NO_ERROR:
				printf("Level: No Error\r\n");
				break;
			case DEBUG:
				printf("Level: Debug\r\n");
				break;
			case INFORMATION:
				printf("Level: Information\r\n");
				break;
			case MINIMAL:
				printf("Level: Minimal\r\n");
				break;
			case CAUTION:
				printf("Level: Caution\r\n");
				break;
			case SEVERE:
				printf("Level: Severe\r\n");
				break;
			case FATAL:
				printf("Level: Fatal\r\n");
				break;
			default:
				break;
		}
		switch(msg->Diagnostic_Message)   
		{
			case NO_ERROR:
				printf("Diag: No Error.\r\n");
				break;
			case INITIALIZING:
				printf("Diag: Initializing.\r\n");
				break;
			case DROPPING_PACKETS:
				printf("Diag: Dropping Packets.\r\n");
				break;
			case MISSING_HEARTBEATS:
				printf("Diag: Missing Heartbeats.\r\n");
				break;
			case DEVICE_NOT_AVAILABLE:
				printf("Diag: Device Not Available.\r\n");
				break;
			case GENERAL_ERROR:
				printf("Diag: General Error\r\n");
				break;
			default:
				break;
		}
		printf("%s\r\n",msg->Description.c_str());
	}
}
int main(int argc, char **argv)
{
  int INITIALIZED = 0;
  ros::init(argc, argv, "Diagnostic Node");
  ros::NodeHandle nh;
  //nh.getParam("target_count",target_count);
  //nh.getParam("diag_device",DIAG_device);
  //nh.getParam("baudrate",Baud_Rate);
  nh.getParam("Output_Level",Output_Level);
  cout << "Output Level: " <<  Output_Level << endl;
  
  ros::Subscriber Sub_Diagnostic = nh.subscribe<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Mapping_Diagnostic",1000,Diagnostic_Callback);
  ros::Subscriber Sub_Diagnostic2 = nh.subscribe<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Sonic_Controller_Diagnostic",1000,Diagnostic_Callback);
  //ros::Subscriber Sub_ICARUS_Probe_Command_Callback = nh.subscribe("ICARUS_Probe_Command", 1000, ICARUS_Probe_Command_Callback);
  //ros::Subscriber Pub_Rover_Control = nh.subscribe<sensor_msgs::Joy>("ICARUS_Rover_Control",1000,ICARUS_Rover_Control_Callback);
  ros::Rate loop_rate(100);
	std::clock_t    start;
  while( ros::ok())
	{
    
	  start = std::clock();
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  try
	  {
      
      }
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
      
	  }
  }
  
  
}
