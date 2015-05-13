//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
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

using namespace std;



//Communication Variables

//Operation Variables
string Mode = "";

//Program Variables
double dtime = 0.0;

void ICARUS_Rover_Pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	cout << "Got a Pose" << endl;
	printf("Got a Pose\r\n");
}
void ICARUS_Sonar_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	cout << "Got a Scan" << endl;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mapping");
  ros::NodeHandle nh;
  
  nh.getParam("Mode",Mode);
  
    
  ros::Publisher Pub_ICARUS_Mapping_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Mapping_Diagnostic",1000);
  ros::Rate loop_rate(100);
  std::clock_t    start;
  
  if(Mode.compare("Sim") == 0)
  {
    ros::Publisher Pub_ICARUS_OccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>("ICARUS_SimOccupancyGrid",1000);
    ros::Subscriber Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("ICARUS_SimRover_Pose",1000,ICARUS_Rover_Pose_Callback);
    cout << Sub_Rover_Pose << endl;
    ros::Subscriber Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("ICARUS_SimSonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
    cout << "Sim Mode" << endl;
  }
  else if(Mode.compare("Live") == 0)
  {
    ros::Publisher Pub_ICARUS_OccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>("ICARUS_OccupancyGrid",1000);
    ros::Subscriber Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("ICARUS_Rover_Pose",1000,ICARUS_Rover_Pose_Callback);
    ros::Subscriber Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("ICARUS_Sonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
    cout << "Live Mode" << endl;
  }	
  
  ::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
  ICARUS_Diagnostic.header.frame_id = "ICARUS_Mapping_Diagnostic";
  ICARUS_Diagnostic.System = ROVER;
  ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
  ICARUS_Diagnostic.Component = MAPPING_NODE;
  
 
	while(ros::ok())
	{
    
	  start = std::clock();
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  try
	  {
	    
	    dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
        	
             //ICARUS Diagnostics Publisher
            ICARUS_Diagnostic.header.stamp = ros::Time::now();
            Pub_ICARUS_Mapping_Diagnostic.publish(ICARUS_Diagnostic);
	  }
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
            
            //ICARUS Diagnostics Publisher
            ICARUS_Diagnostic.header.stamp = ros::Time::now();
            ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
            ICARUS_Diagnostic.Level = FATAL;
            ICARUS_Diagnostic.Diagnostic_Message = GENERAL_ERROR;
            ICARUS_Diagnostic.Description = ex.what();
            Pub_ICARUS_Mapping_Diagnostic.publish(ICARUS_Diagnostic);
	  }
  }
  ICARUS_Diagnostic.header.stamp = ros::Time::now();
  ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
  ICARUS_Diagnostic.Level = SEVERE;
  ICARUS_Diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
  ICARUS_Diagnostic.Description = "Node Closed.";
  Pub_ICARUS_Mapping_Diagnostic.publish(ICARUS_Diagnostic);
  
}
