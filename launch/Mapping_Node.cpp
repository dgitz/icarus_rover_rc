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
#include <boost/algorithm/string.hpp>
//#include "icarus_rover_rc/ICARUS_Diagnostic.h"
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
#define PI 3.14159265359
using namespace std;
//Function Prototypes
void initialize_occupancygrid();
void reset_occupancygrid();
void read_SonarModel_LUT();
//Communication Variables

//Operation Variables
string Mode = "";
int Beams_Per_Sonar = 0;
double Sonar_Beamwidth = 0.0;
double Sonar_MaxDistance = 0.0;
struct grid_cell cell_origin;
int New_Pose = 0;
int New_Scan = 0;
int Grid_Finished = 1;

struct GridCell
{
	int Value;
	int ID;
	int Updated;
	double Probability;
	double Center_Northing_m;
	double Center_Easting_m;
	int Changed;
	int Parent; //ID
	
};
struct SonarModel_Point
{
	int index;
	double value;
};

sensor_msgs::LaserScan SonarScan;
geometry_msgs::Pose2D RoverPose;
//Program Variables
double dtime = 0.0;
double Pose_X;
double Pose_Y;
double Pose_Theta;
bool map_initialized = false;
const int grid_width = 251; //Cell Count, should be odd
const int grid_height = grid_width; //Cell Count, should be odd
double bottom_left_X = -15.0;
double bottom_left_Y = bottom_left_X;
double top_right_X = -1.0*bottom_left_X;
double top_right_Y = top_right_X;
double grid_resolution = (top_right_Y - bottom_left_Y)/grid_width;
int SonarModel_AngleSize = 0;
int SonarModel_DistanceSize = 0;
std::vector<SonarModel_Point> SonarModel_Angle;
std::vector<SonarModel_Point> SonarModel_Distance;


GridCell OccupancyGrid[grid_width][grid_height];
void read_SonarModel_LUT()
{
	ifstream angle_file("/home/linaro/catkin_ws/src/icarus_rover_rc/include/icarus_rover_rc/SonarModel_Angle.csv");
	string line;
	
	if(angle_file.is_open())
	{
		int i = 0;
		while(getline(angle_file,line))
		{
			
			std::vector<std::string> strs;
			boost::split(strs,line,boost::is_any_of(","));
			SonarModel_Point newPoint;
			newPoint.index = atoi(strs.at(0).c_str());
			newPoint.value = atof(strs.at(1).c_str());
			SonarModel_Angle.push_back(newPoint);
			i++;
		}
		SonarModel_AngleSize = i;
		angle_file.close();
		for(i = 0; i < SonarModel_AngleSize;i++)
		{
			printf("Ai:%d V:%f\r\n",SonarModel_Angle.at(i).index,SonarModel_Angle.at(i).value);
		}
	}
	ifstream distance_file("/home/linaro/catkin_ws/src/icarus_rover_rc/include/icarus_rover_rc/SonarModel_Distance.csv");
	if(distance_file.is_open())
	{
		int i = 0;
		while(getline(distance_file,line))
		{
			
			std::vector<std::string> strs;
			boost::split(strs,line,boost::is_any_of(","));
			SonarModel_Point newPoint;
			newPoint.index = atoi(strs.at(0).c_str());
			newPoint.value = atof(strs.at(1).c_str());
			SonarModel_Distance.push_back(newPoint);
			i++;
		}
		SonarModel_DistanceSize = i;
		distance_file.close();
		for(i = 0; i < SonarModel_DistanceSize;i++)
		{
			printf("Di:%d V:%f\r\n",SonarModel_Distance.at(i).index,SonarModel_Distance.at(i).value);
		}
	}
	
}
void initialize_occupancygrid()
{
	double start_x = bottom_left_X;
	double start_y = top_right_Y;
	double curx = start_x + grid_resolution/2.0;
	double cury = start_y - grid_resolution/2.0;
	for(int i = 0; i < grid_width;i++)
	{
		for(int j = 0; j < grid_height;j++)
		{
			OccupancyGrid[i][j].Updated = 0;
			OccupancyGrid[i][j].Probability = 0.5;
			OccupancyGrid[i][j].Value = 50;
			OccupancyGrid[i][j].ID = i*grid_width + j;
			OccupancyGrid[i][j].Center_Northing_m = curx;
			OccupancyGrid[i][j].Center_Easting_m = cury;
			cury = cury - grid_resolution;
			//Debugging
			//if(OccupancyGrid[i][j].ID % 2 == 0) { OccupancyGrid[i][j].Value = 255; }
			//else { OccupancyGrid[i][j].Value = 0; }
		}
		curx = curx + grid_resolution;
		cury = start_y - grid_resolution/2.0;
	}
}
void update_occupancygrid()
{
	Grid_Finished = 0;
	for(int i = 0; i < grid_width;i++)
	{
		for(int j = 0; j < grid_height;j++)
		{
			OccupancyGrid[i][j].Updated = 0;
			//OccupancyGrid[i][j].Value = 0;
		}
	}
	int beam_count = SonarScan.ranges.size();
	double angle = RoverPose.theta + SonarScan.angle_min;
	//printf("START\r\n");
	for(int s = 0; s < beam_count;s++)
	{
		double distance = SonarScan.ranges[s];
		int beam_intercepted = 0;
		if(SonarScan.ranges[s] < 0.0)
		{
			beam_intercepted = 0;
			distance = Sonar_MaxDistance; 
		}
		else
		{
			beam_intercepted = 1;
		}
		while(distance > 0.0)
		{
		double p_x = distance*sin(angle) + RoverPose.x;
		double p_y = distance*cos(angle) + RoverPose.y;
		int j = round((p_x/grid_resolution) + grid_width/2.0)-1;
		int i = round((p_y/grid_resolution) + grid_height/2.0)-1;

		if((i >= 0) and (i < grid_width) and (j >= 0) and (j < grid_height))
		{
			if(OccupancyGrid[i][j].Updated == 0)
			{
				double temp;
				int distance_index;
				int angle_index;
				int beam_index = s % Beams_Per_Sonar;
				double angle = (beam_index-(Beams_Per_Sonar/2.0)) * ((Sonar_Beamwidth/2.0)/(Beams_Per_Sonar/2.0));
				
				if(beam_intercepted == 0)
				{
					distance_index = 0;
					angle_index = 1000;
				}
				else
				{
					 distance_index = (int)1000.0*(distance/SonarScan.ranges[s]);
					 angle_index = (int)1000.0*(fabs(angle)/(Sonar_Beamwidth/2.0));
				}
				asdf
				if(distance_index < 0) { distance_index = 0;}
				if(distance_index > (SonarModel_DistanceSize-1)) {distance_index = SonarModel_DistanceSize-1;}
				if(angle_index < 0) { angle_index = 0;}
				if(angle_index > (SonarModel_AngleSize-1)) {angle_index = SonarModel_AngleSize-1;}
				double P_distance = SonarModel_Distance.at(distance_index).value;
				double P_angle = SonarModel_Angle.at(angle_index).value;
				
				double P = P_distance * P_angle;
				OccupancyGrid[i][j].Probability = (P*OccupancyGrid[i][j].Probability)/((P*OccupancyGrid[i][j].Probability)+((1.0-P)*OccupancyGrid[i][j].Probability));
				OccupancyGrid[i][j].Value = (int)(100.0*OccupancyGrid[i][j].Probability);
				//if(s == 450)
				//{
					//printf("%d/%d t: %f i: %d D: %f d:%f P: %f P: %f V: %d\r\n",i,j,temp,distance_index,SonarScan.ranges[s],distance,P_distance,OccupancyGrid[i][j].Probability,OccupancyGrid[i][j].Value);
				//}
				//printf("i: %d j: %d x:%f y:%f theta:%f/%f  res: %f",i,j,p_x,p_y,angle,angle*180.0/PI,grid_resolution);
				//OccupancyGrid[i][j].Value = 255;
				
				/*if((distance >= .9*SonarScan.ranges[s]))
				{
					OccupancyGrid[i][j].Value = OccupancyGrid[i][j].Value + 1;
					if(OccupancyGrid[i][j].Value > 255) {OccupancyGrid[i][j].Value = 255; }
				}
				else
				{
					OccupancyGrid[i][j].Value = OccupancyGrid[i][j].Value - 1;
					if(OccupancyGrid[i][j].Value < 0) {OccupancyGrid[i][j].Value = 0;}
				}
				OccupancyGrid[i][j].Updated = 1;*/
			}
		}
		
		distance = distance - grid_resolution;
	}
		
		angle = angle + SonarScan.angle_increment;
		//if(angle < SonarScan.angle_min) { break; }
	}
	//printf("FINISH\r\n");
	Grid_Finished = 1;
}
void print_occupancygrid()
{
	printf("START\r\n");
	for(int i = 0; i < grid_width; i++)
	{
		for(int j = 0; j < grid_height; j++)
		{
			printf("i:%dj:%dN:%fE:%fv:%d   ",i,j,OccupancyGrid[i][j].Center_Northing_m,OccupancyGrid[i][j].Center_Easting_m,OccupancyGrid[i][j].Value);
		}
		printf("\r\n");
	}
	printf("FINISH\r\n");
}
void ICARUS_Rover_Pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	RoverPose.x = msg->x;
	RoverPose.y = msg->y;
	RoverPose.theta = msg->theta;
	New_Pose = 1;
	//printf("Got a Pose x: %f y: %f theta: %f\r\n",msg->x,msg->y,msg->theta);
}
void ICARUS_Sonar_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//cout << "Got a Scan" << endl;
	SonarScan.angle_min = msg->angle_min;
	SonarScan.angle_max = msg->angle_max;
	SonarScan.angle_increment = msg->angle_increment;
	SonarScan.range_min = msg->range_min;
	SonarScan.range_max = msg->range_max;
	SonarScan.ranges = msg->ranges;
	New_Scan = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mapping_Node");
  ros::NodeHandle nh("~");
  
  nh.getParam("Operation_Mode",Mode);
  nh.getParam("Beams_Per_Sonar",Beams_Per_Sonar);
  nh.getParam("Sonar_Beamwidth",Sonar_Beamwidth);
	nh.getParam("Sonar_MaxDistance",Sonar_MaxDistance);
  //ros::Publisher Pub_ICARUS_Mapping_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Mapping_Diagnostic",1000);
  ros::Rate loop_rate(10);
  std::clock_t    start;
  ros::Publisher Pub_ICARUS_OccupancyGrid;   
  ros::Subscriber Sub_Rover_Pose;
  ros::Subscriber Sub_Sonar_Scan;

 if(Mode.compare("SIM") == 0)
  {
    Pub_ICARUS_OccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>("ICARUS_SimOccupancyGrid",1000);
    Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Pose",1000,ICARUS_Rover_Pose_Callback);
    //Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Matlab_Node/ICARUS_SimSonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
    cout << "Sim Mode" << endl;
  }
  else if(Mode.compare("LIVE") == 0)
  {
    Pub_ICARUS_OccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>("ICARUS_OccupancyGrid",1000);
    Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Motion_Controller_Node/ICARUS_Rover_Pose",1000,ICARUS_Rover_Pose_Callback);
	//Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Sonic_Controller_Node/ICARUS_Sonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
    cout << "Live Mode" << endl;
  }	
  Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Sonic_Controller_Node/ICARUS_Sonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
	initialize_occupancygrid();
  //::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
  nav_msgs::OccupancyGrid ros_OccupancyGrid;
  /*ICARUS_Diagnostic.header.frame_id = "ICARUS_Mapping_Diagnostic";
  ICARUS_Diagnostic.System = ROVER;
  ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
  ICARUS_Diagnostic.Component = MAPPING_NODE;*/
	read_SonarModel_LUT();
	while(ros::ok())
	{
    
		start = std::clock();
		ros::spinOnce();
		loop_rate.sleep();
		
		//printf("bx: %f by: %f tx: %f ty: %f\r\n",bottom_left_x,bottom_left_y,top_right_x,top_right_y);
		try
		{
			if((Grid_Finished == 1) and (New_Scan == 1))
			{	
				Grid_Finished = 0;
				update_occupancygrid();
				ros_OccupancyGrid.header.stamp = ros::Time::now();
				ros_OccupancyGrid.info.resolution = grid_resolution;
				ros_OccupancyGrid.info.width = grid_width;
				ros_OccupancyGrid.info.height = grid_height;
				ros_OccupancyGrid.info.origin.position.x = bottom_left_X;
				ros_OccupancyGrid.info.origin.position.y = bottom_left_Y;
				ros_OccupancyGrid.data.resize(grid_width*grid_height);
				for(int i = 0; i < grid_width; i++)
				{
					for(int j = 0; j < grid_height;j++)
					{
						ros_OccupancyGrid.data[i*grid_width+j] = OccupancyGrid[i][j].Value;
					}
				}
				//ros_OccupancyGrid.origin.
				Pub_ICARUS_OccupancyGrid.publish(ros_OccupancyGrid);
				New_Scan = 0;
				Grid_Finished = 1;
				
			}
			dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
			
			//print_occupancygrid();
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
}/*
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
*/
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

