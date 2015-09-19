//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
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
#include "icarus_rover_rc/Mapping_Node.h"
#include "icarus_rover_rc/Definitions.h"
//#include "icarus_rover_rc/ICARUS_Diagnostic.h"
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
using namespace std;


//Communication Variables

//Operation Variables
string Mode = "";
struct grid_cell cell_origin;



//Program Variables
double dtime = 0.0;
double Pose_X;
double Pose_Y;
double Pose_Theta;
bool map_initialized = false;
int grid_width = 13; //Cell Count, should be odd
int grid_height = 9; //Cell Count, should be odd
double grid_size = 5.0; //Meters
double bottom_left_X = (-1.0*grid_size*grid_width/2.0);
double bottom_left_Y = (-1.0*grid_size*grid_height/2.0);
double top_right_X = (1.0*grid_size*grid_width/2.0);
double top_right_Y = (1.0*grid_size*grid_height/2.0);


void ICARUS_Rover_Pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	printf("Got a Pose x: %f y: %f theta: %f\r\n",msg->x,msg->y,msg->theta);
}
void ICARUS_Sonar_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	cout << "Got a Scan" << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mapping_Node");
  ros::NodeHandle nh("~");
  cell_origin.x = 0;
  cell_origin.y = 0;
  cell_origin.X = 0.0;
  cell_origin.Y = 0.0;
  cell_origin.index = get_index_from_cell(cell_origin.x,cell_origin.y);
  printf("bx: %f by: %f tx: %f ty: %f\r\n",bottom_left_X,bottom_left_Y,top_right_X,top_right_Y);
  for (int i = 0; i <= 0; i++)
  {
	for(int j = -60; j <= 60; j++)
	{
		double X = i * grid_size/8.0;
		double Y = j * grid_size/8.0;
		grid_cell cell;
		cell = find_cell( cell, cell_origin,X, Y,0);
		printf("X: %f Y: %f x: %d y: %d stat: %d\r\n",X,Y,cell.x,cell.y,cell.status);
	}
  }
  nh.getParam("Operation_Mode",Mode);
  
    
  //ros::Publisher Pub_ICARUS_Mapping_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Mapping_Diagnostic",1000);
  ros::Rate loop_rate(100);
  std::clock_t    start;
  ros::Publisher Pub_ICARUS_OccupancyGrid;  
  ros::Publisher Pub_ICARUS_Rover_Odom;    
  ros::Subscriber Sub_Rover_Pose;
  ros::Subscriber Sub_Sonar_Scan;

 if(Mode.compare("SIM") == 0)
  {
    Pub_ICARUS_OccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>("ICARUS_SimOccupancyGrid",1000);
    Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Pose",1000,ICARUS_Rover_Pose_Callback);
    cout << Sub_Rover_Pose << endl;
    Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Matlab_Node/ICARUS_SimSonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
    Pub_ICARUS_Rover_Odom = nh.advertise<nav_msgs::Odometry>("ICARUS_SimRover_Odom",1000);
    cout << "Sim Mode" << endl;
  }
  else if(Mode.compare("LIVE") == 0)
  {
    Pub_ICARUS_OccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>("ICARUS_OccupancyGrid",1000);
    Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Motion_Controller_Node/ICARUS_Rover_Pose",1000,ICARUS_Rover_Pose_Callback);
    Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Sonic_Controller_Node/ICARUS_Sonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
    Pub_ICARUS_Rover_Odom = nh.advertise<nav_msgs::Odometry>("ICARUS_Rover_Odom",1000);
    cout << "Live Mode" << endl;
  }	
 
  //::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
  nav_msgs::OccupancyGrid OccupancyGrid;
  /*ICARUS_Diagnostic.header.frame_id = "ICARUS_Mapping_Diagnostic";
  ICARUS_Diagnostic.System = ROVER;
  ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
  ICARUS_Diagnostic.Component = MAPPING_NODE;*/

	while(ros::ok())
	{
    
		start = std::clock();
		ros::spinOnce();
		loop_rate.sleep();
		
		//printf("bx: %f by: %f tx: %f ty: %f\r\n",bottom_left_x,bottom_left_y,top_right_x,top_right_y);
		try
		{

			dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
			//Pub_ICARUS_OccupancyGrid.publish(OccupancyGrid);
			
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
            Pub_ICARUS_Mapping_Diagnostic.publish(ICARUS_Diagnostic);*/
		}
	}
}
int get_index_from_cell(int x, int y)
{
	double x_index_calc; 
	double y_index_calc;
	y_index_calc = (y+floor(grid_height/2.0))*grid_width;
	x_index_calc = x+floor(grid_width/2.0);
	return y_index_calc + x_index_calc;
}
grid_cell find_cell(grid_cell mycell, grid_cell map_origin,double X, double Y, int value)
{
	grid_cell new_cell;
	new_cell.X = X;
	new_cell.Y = Y;
	if (new_cell.X < bottom_left_X)
	{
		new_cell.status = 0;
		return new_cell;
	}
	else if(new_cell.X > top_right_X)
	{
		new_cell.status = 0;
		return new_cell;
	}
	else
	{
		new_cell.x = floor((new_cell.X-map_origin.X+grid_size/2.0)/grid_size);
	}
	if (new_cell.Y < bottom_left_Y)
	{
		new_cell.status = 0;
		return new_cell;
	}
	else if(new_cell.Y > top_right_Y)
	{
		new_cell.status = 0;
		return new_cell;
	}
	else
	{
		new_cell.y = floor((map_origin.Y-new_cell.Y+grid_size/2.0)/grid_size);
	}
	if(new_cell.y > grid_height/2.0) { new_cell.y = floor(grid_height/2.0); }
	if(new_cell.x > grid_width/2.0) { new_cell.x = floor(grid_width/2.0); }
	new_cell.status = 1;
	return new_cell;
}

/*
int grid_width = 11; //Cell Count, should be odd
int grid_height = 11; //Cell Count, should be odd
double grid_size = 5.0; //Meters
double bottom_left_x = -1.0*grid_size*grid_width/2.0;
double bottom_left_y = -1.0*grid_size*grid_height/2.0;
double top_right_x = 1.0*grid_size*grid_width/2.0;
double top_right_y = 1.0*grid_size*grid_height/2.0;
*/
  /*ICARUS_Diagnostic.header.stamp = ros::Time::now();
  ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
  ICARUS_Diagnostic.Level = SEVERE;
  ICARUS_Diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
  ICARUS_Diagnostic.Description = "Node Closed.";
  Pub_ICARUS_Mapping_Diagnostic.publish(ICARUS_Diagnostic);*/

