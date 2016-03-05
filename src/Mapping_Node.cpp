//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <ros/console.h>
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
#include <assert.h>
#define PI 3.14159265359
using namespace std;
//Function Prototypes
void initialize_occupancygrid();
void reset_occupancygrid();
void read_SonarModel_LUT();
int path_search();
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
int Path_Finished = 0;

struct GridCell
{
	int Value;
	int ID;
	int Updated;
	double Probability;
	double Center_Northing_m;
	double Center_Easting_m;
	int Changed;
	
};

struct Node
{
	int Parent_ID;
	int ID;
	double x,y;
	int i,j;
	double f,g,h;
};

std::vector<Node> OPEN_LIST;
std::vector<Node> CLOSED_LIST;
std::vector<Node> PATH;
struct SonarModel_Point
{
	int index;
	double value;
};
sensor_msgs::LaserScan SonarScan;
geometry_msgs::Pose2D RoverPose;
geometry_msgs::Pose2D GoalPose;
//Program Variables
double dtime = 0.0;
double Pose_X;
double Pose_Y;
double Pose_Theta;
int Pose_Valid = 0;
bool map_initialized = false;
const int grid_width = 100; //Cell Count, should be odd
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
void ICARUS_SimulationState_Callback(const std_msgs::Int32::ConstPtr& msg)
{
	printf("Mapping_Node ICARUS_SimulationState_Callback Started.\r\n");

	try
	{
		int state = msg->data;
		if(state == STATUS_GENELIST_READY) 
		{
			//printf("Resetting Map.\r\n");
			reset_occupancygrid();
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
	printf("Mapping_Node ICARUS_SimulationState_Callback Completed.\r\n");
}
void read_SonarModel_LUT()
{
	printf("Mapping_Node read_SonarModel_LUT Started.\r\n");
	try
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
			/*for(i = 0; i < SonarModel_AngleSize;i++)
			{
				printf("Ai:%d V:%f\r\n",SonarModel_Angle.at(i).index,SonarModel_Angle.at(i).value);
			}*/
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
			/*for(i = 0; i < SonarModel_DistanceSize;i++)
			{
				printf("Di:%d V:%f\r\n",SonarModel_Distance.at(i).index,SonarModel_Distance.at(i).value);
			}*/
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
	printf("Mapping_Node read_SonarModel_LUT Completed.\r\n");
	
}
void initialize_occupancygrid()
{
	printf("Mapping_Node initialize_occupancygrid Started.\r\n");
	try
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
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
	printf("Mapping_Node initialize_occupancygrid Completed.\r\n");
}
void reset_occupancygrid()
{
	printf("Mapping_Node reset_occupancygrid Started.\r\n");
	try
	{
		for(int i = 0; i < grid_width;i++)
		{
			for(int j = 0; j < grid_height;j++)
			{
				OccupancyGrid[i][j].Updated = 0;
				OccupancyGrid[i][j].Probability = 0.5;
				OccupancyGrid[i][j].Value = 50;
			}
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
	//printf("Mapping_Node reset_occupancygrid Completed.\r\n");
			
}
void update_occupancygrid()
{
	//printf("Mapping_Node update_occupancygrid Started.\r\n");
	try
	{
		if(Pose_Valid == 1)
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
				else if(SonarScan.ranges[s] > Sonar_MaxDistance)
				{
					beam_intercepted = 0;
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
				if((RoverPose.x > bottom_left_X)  and (RoverPose.x < top_right_X) and (RoverPose.y > bottom_left_Y) and (RoverPose.y < top_right_Y))
				{
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
							if(distance_index < 0) { distance_index = 0;}
							if(distance_index > (SonarModel_DistanceSize-1)) {distance_index = SonarModel_DistanceSize-1;}
							if(angle_index < 0) { angle_index = 0;}
							if(angle_index > (SonarModel_AngleSize-1)) {angle_index = SonarModel_AngleSize-1;}
							double P_distance = SonarModel_Distance.at(distance_index).value;
							double P_angle = SonarModel_Angle.at(angle_index).value;
							
							double P = P_distance * P_angle;
							OccupancyGrid[i][j].Probability = (P*OccupancyGrid[i][j].Probability)/((P*OccupancyGrid[i][j].Probability)+((1.0-P)*OccupancyGrid[i][j].Probability));
							if(OccupancyGrid[i][j].Probability < 0.0) { OccupancyGrid[i][j].Probability = 0.0;}
							if(OccupancyGrid[i][j].Probability > 1.0) { OccupancyGrid[i][j].Probability = 1.0;}
							OccupancyGrid[i][j].Value = (int)(100.0*OccupancyGrid[i][j].Probability);
							if(OccupancyGrid[i][j].Value < 0) { OccupancyGrid[i][j].Value = 0;}
							if(OccupancyGrid[i][j].Value > 100) {OccupancyGrid[i][j].Value = 100;}
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
				}
				
				distance = distance - grid_resolution;
			}
				
				angle = angle + SonarScan.angle_increment;
				//if(angle < SonarScan.angle_min) { break; }
			}
			//printf("FINISH\r\n");
			Grid_Finished = 1;
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
	//printf("Mapping_Node update_occupancygrid Completed.\r\n");
}
void print_occupancygrid()
{
	try
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
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
}
void ICARUS_Rover_Pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	//printf("Mapping_Node ICARUS_Rover_Pose_Callback Started.\r\n");
	//printf("Mapping_Node ICARUS_Rover_Pose_Callback Started.\r\n");
	try
	{
		if((abs(msg->x) < 15.0) and (abs(msg->y) < 15.0))
		{
			RoverPose.x = msg->x;
			RoverPose.y = msg->y;
			RoverPose.theta = msg->theta;
			New_Pose = 1;
			Pose_Valid = 1;
		}
		else
		{
			Pose_Valid = 0;
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
	//printf("Mapping_Node ICARUS_Rover_Pose_Callback Completed.\r\n");
	//printf("Got a Pose x: %f y: %f theta: %f\r\n",msg->x,msg->y,msg->theta);
}
void ICARUS_Rover_Goal_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	//printf("Mapping_Node ICARUS_Rover_Goal_Callback Started.\r\n");
	try
	{
		if((abs(msg->x) < 15.0) and (abs(msg->y) < 15.0) and (abs(msg->theta) < 10.0))
		{
			GoalPose.x = msg->x;
			GoalPose.y = msg->y;
			GoalPose.theta = msg->theta;
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
	//printf("Mapping_Node ICARUS_Rover_Goal_Callback Completed.\r\n");
	//printf("Got a Goal x: %f y: %f theta: %f\r\n",msg->x,msg->y,msg->theta);
}
void ICARUS_Sonar_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//printf("Mapping_Node ICARUS_Sonar_Scan_Callback Started.\r\n");
	try
	{
		if(Pose_Valid == 1)
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
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
	//printf("Mapping_Node ICARUS_Sonar_Scan_Callback Completed.\r\n");
}

int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "Mapping_Node");
		ros::NodeHandle nh("~");

		nh.getParam("Operation_Mode",Mode);
		nh.getParam("Beams_Per_Sonar",Beams_Per_Sonar);
		nh.getParam("Sonar_Beamwidth",Sonar_Beamwidth);
		nh.getParam("Sonar_MaxDistance",Sonar_MaxDistance);
		//printf("Got Debug Mode: %d\r\n",0);
		//ros::Publisher Pub_ICARUS_Mapping_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Mapping_Diagnostic",1000);
		ros::Rate loop_rate(3);
		std::clock_t    start;
		ros::Publisher Pub_ICARUS_OccupancyGrid;   
		ros::Subscriber Sub_Rover_Pose;
		ros::Subscriber Sub_Sonar_Scan;
		ros::Subscriber Sub_Rover_Goal;
		ros::Subscriber Sub_Simulation_State;
		ros::Publisher Pub_Rover_GlobalPath;
		if(Mode.compare("SIM") == 0)
		{
			Pub_ICARUS_OccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>("ICARUS_SimOccupancyGrid",1000);
			Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Pose",1000,ICARUS_Rover_Pose_Callback);
			Sub_Rover_Goal = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Goal",1000,ICARUS_Rover_Goal_Callback);
			Sub_Simulation_State = nh.subscribe<std_msgs::Int32>("/Matlab_Node/ICARUS_State",1000,ICARUS_SimulationState_Callback);
			//Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Matlab_Node/ICARUS_SimSonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
			cout << "Mapping Node: Sim Mode" << endl;
		}
		else if(Mode.compare("LIVE") == 0)
		{
			Pub_ICARUS_OccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>("ICARUS_OccupancyGrid",1000);
			Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Motion_Controller_Node/ICARUS_Rover_Pose",1000,ICARUS_Rover_Pose_Callback);
			Sub_Rover_Goal = nh.subscribe<geometry_msgs::Pose2D>("/Robot_Controller_Node/Rover_Goal",1000,ICARUS_Rover_Goal_Callback);
			//Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Sonic_Controller_Node/ICARUS_Sonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
			cout << "Mapping Node: Live Mode" << endl;
		}	
		Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Sonic_Controller_Node/ICARUS_Sonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
		Pub_Rover_GlobalPath = nh.advertise<nav_msgs::Path>("ICARUS_Rover_GlobalPath",1000);
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
			//printf("Mapping_Node ROS Spin.\r\n");
			start = std::clock();
			ros::spinOnce();
			loop_rate.sleep();
			
			//printf("bx: %f by: %f tx: %f ty: %f\r\n",bottom_left_x,bottom_left_y,top_right_x,top_right_y);
			try
			{
				
				if((Grid_Finished == 1) and (New_Scan == 1))
				{	
					//printf("Mapping_Node Grid Finished and New Scan.\r\n");
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
					printf("Mapping_Node Published Grid\r\n");
					New_Scan = 0;
					Grid_Finished = 1;
					if(Pose_Valid == 1)
					{
						if(path_search() == 1)
						{
							//printf("Mapping_Node Path Search Complete\r\n");
							//("PATH FOUND!");
							std::reverse(PATH.begin(),PATH.end());
							//printf("Mapping_Node Reversed Path\r\n");
							nav_msgs::Path GlobalPath;
							GlobalPath.header.stamp = ros::Time::now();
							GlobalPath.header.frame_id = "map";
							GlobalPath.poses.resize(PATH.size());
							//os_OccupancyGrid.data.resize(grid_width*grid_height);
							for(int k = 0; k < PATH.size();k++)
							{
								//printf("%d(%d,%d,%f,%f,%f)->",PATH.at(k).ID,PATH.at(k).i,PATH.at(k).j,PATH.at(k).x,PATH.at(k).y,OccupancyGrid[PATH.at(k).i][PATH.at(k).j].Probability);
								GlobalPath.poses[k].header.stamp = ros::Time::now();
								GlobalPath.poses[k].pose.position.x = PATH.at(k).x;
								GlobalPath.poses[k].pose.position.y = PATH.at(k).y;
								
							}
							//printf("PATH COMPLETE.\r\n");
							Pub_Rover_GlobalPath.publish(GlobalPath);
							//printf("Mapping_Node Published Path\r\n");
						}
						else
						{
							//printf("Mapping_Node Path Search Failed\r\n");
						}
					}
					
				}
				dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
				
				//print_occupancygrid();
			}
			catch(exception& e)
			{
				cout << "RC Node: " << e.what() << endl;
				ROS_ERROR("%s",e.what());
			}
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
}
int path_search()
{
	//printf("Mapping_Node Path Search Started\r\n");
	try
	{
		OPEN_LIST.clear();
		CLOSED_LIST.clear();
		PATH.clear();
		Node NewNode;
		
		/*
		double p_x = distance*sin(angle) + RoverPose.x;
			double p_y = distance*cos(angle) + RoverPose.y;
			int j = round((p_x/grid_resolution) + grid_width/2.0)-1;
			int i = round((p_y/grid_resolution) + grid_height/2.0)-1;
		*/
		NewNode.x = RoverPose.x;
		NewNode.y = RoverPose.y;
		NewNode.i = round((NewNode.y/grid_resolution) + grid_height/2.0)-1;
		NewNode.j = round((NewNode.x/grid_resolution) + grid_width/2.0)-1;
		NewNode.ID = (NewNode.i*grid_width) + NewNode.j;
		NewNode.Parent_ID = -1; //No Parent here
		NewNode.f = 0.0;
		NewNode.g = 0.0;
		NewNode.h = 0.0;
		OPEN_LIST.push_back(NewNode);
		int Goal_i = round((GoalPose.y/grid_resolution) + grid_height/2.0) -1;
		int Goal_j = round((GoalPose.x/grid_resolution) + grid_width/2.0) -1;
		//int Goal_i = 80;
		//int Goal_j = 50;
		int Start_ID = NewNode.ID;
		//printf("Si: %d Sj: %d SID: %d Gi: %d Gj: %d\r\n",NewNode.i,NewNode.j,NewNode.ID,Goal_i,Goal_j);
		int iteration_counter = 0;
		int path_found = 0;
		while(OPEN_LIST.size() > 0)
		{
			//printf("\r\n\r\n\r\nI: %d OPEN:\r\n",iteration_counter);
			/*for(int l = 0; l < OPEN_LIST.size();l++)
			{
				printf(" ID: %d i: %d j: %d\r\n",OPEN_LIST.at(l).ID,OPEN_LIST.at(l).i,OPEN_LIST.at(l).j);
			}*/
			/*printf("I: %d CLOSED:\r\n",iteration_counter);
			for(int l = 0; l < CLOSED_LIST.size();l++)
			{
				printf(" ID: %d i: %d j: %d P: %d\r\n",CLOSED_LIST.at(l).ID,CLOSED_LIST.at(l).i,CLOSED_LIST.at(l).j,CLOSED_LIST.at(l).Parent_ID);
			}*/
			if(iteration_counter > (10*grid_width*grid_width))//I SEARCHED WAY TOO MANY CELLS
			{
				printf("NO PATH FOUND!\r\n");
				return 0;
			}
			//Assume the lowest f Node is the first one, and search the rest to see if there is anyone better.
			double min_f = OPEN_LIST.at(0).f;
			int index = 0;
			for(int i = 0; i < OPEN_LIST.size();i++)
			{
				if(OPEN_LIST.at(i).f < min_f) 
				{ 
					min_f = OPEN_LIST.at(i).f;
					index = i;
				}
			}
			Node q = OPEN_LIST.at(index);
			OPEN_LIST.erase(OPEN_LIST.begin()+index);
			//printf("OPEN LIST SIZE: %d\r\n",OPEN_LIST.size());
			//printf("OPENING ID: %d P: %d\r\n",q.ID,q.Parent_ID);
			std::vector<Node> SUCCESSORS;
			//Add Successors
			if((q.i == 0) || (q.i == grid_width) || (q.j == 0) || (q.j == grid_width)) 
			{ 
				//printf("Border cell, don't check this.\r\n");
				break; 
			} //Check if cell is on border.
			Node NewNode;
			//Add Successor 0
			
			NewNode.Parent_ID = q.ID;
			NewNode.i = q.i+1;
			NewNode.j = q.j-1;
			NewNode.ID = (NewNode.i * grid_width) + NewNode.j;
			SUCCESSORS.push_back(NewNode);
			//Add Successor 1
			NewNode.Parent_ID = q.ID;
			NewNode.i = q.i+1;
			NewNode.j = q.j;
			NewNode.ID = (NewNode.i * grid_width) + NewNode.j;
			SUCCESSORS.push_back(NewNode);
			//Add Successor 2
			NewNode.Parent_ID = q.ID;
			NewNode.i = q.i+1;
			NewNode.j = q.j+1;
			NewNode.ID = (NewNode.i * grid_width) + NewNode.j;
			SUCCESSORS.push_back(NewNode);
			//Add Successor 3
			NewNode.Parent_ID = q.ID;
			NewNode.i = q.i;
			NewNode.j = q.j+1;
			NewNode.ID = (NewNode.i * grid_width) + NewNode.j;
			SUCCESSORS.push_back(NewNode);
			//Add Successor 4
			NewNode.Parent_ID = q.ID;
			NewNode.i = q.i-1;
			NewNode.j = q.j+1;
			NewNode.ID = (NewNode.i * grid_width) + NewNode.j;
			SUCCESSORS.push_back(NewNode);
			//Add Successor 5
			NewNode.Parent_ID = q.ID;
			NewNode.i = q.i-1;
			NewNode.j = q.j;
			NewNode.ID = (NewNode.i * grid_width) + NewNode.j;
			SUCCESSORS.push_back(NewNode);
			//Add Successor 6
			NewNode.Parent_ID = q.ID;
			NewNode.i = q.i-1;
			NewNode.j = q.j-1;
			NewNode.ID = (NewNode.i * grid_width) + NewNode.j;
			SUCCESSORS.push_back(NewNode);
			//Add Successor 7
			NewNode.Parent_ID = q.ID;
			NewNode.i = q.i;
			NewNode.j = q.j-1;
			NewNode.ID = (NewNode.i * grid_width) + NewNode.j;
			SUCCESSORS.push_back(NewNode);
			/*for(int k = 0; k < SUCCESSORS.size();k++)
			{
				printf("S ID: %d (%d,%d),",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
			}
			printf("\r\n");*/
			for(int k = 0; k < SUCCESSORS.size();k++)
			{
				
				//printf("2S ID:%d (%d,%d)\r\n",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
				SUCCESSORS.at(k).x = (double)((double)SUCCESSORS.at(k).j - (grid_width/2.0) + 1.0)*grid_resolution;
				SUCCESSORS.at(k).y = (double)((double)SUCCESSORS.at(k).i - (grid_height/2.0) + 1.0)*grid_resolution;
				double di; 
				double dj;
				di = q.i - SUCCESSORS.at(k).i;
				dj = q.j - SUCCESSORS.at(k).j;
				double d_successor_to_q = pow((pow(di,2.0)+pow(dj,2.0)),0.5);
				//printf("3S ID:%d (%d,%d)\r\n",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
				SUCCESSORS.at(k).g = q.g + d_successor_to_q;
				di = Goal_i - SUCCESSORS.at(k).i;
				dj = Goal_j - SUCCESSORS.at(k).j; 
				double d_successor_to_goal = pow((pow(di,2.0)+pow(dj,2.0)),0.5);
				//printf("4S ID:%d (%d,%d)\r\n",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
				
				int add_successor = 1;
				if(( SUCCESSORS.at(k).i >= 0 ) and (SUCCESSORS.at(k).i < grid_width) and (SUCCESSORS.at(k).j >= 0) and (SUCCESSORS.at(k).j < grid_height))
				{
					SUCCESSORS.at(k).h = d_successor_to_goal;
					//printf("Here1\r\n");
					SUCCESSORS.at(k).f = SUCCESSORS.at(k).g + SUCCESSORS.at(k).h;
					if(OccupancyGrid[SUCCESSORS.at(k).i][SUCCESSORS.at(k).j].Probability > 0.5){ add_successor = 0; }

				}
				else
				{
					add_successor = 0;
				}
			
				/*
					int Value;
	int ID;
	int Updated;
	double Probability;
	double Center_Northing_m;
	double Center_Easting_m;
	int Changed;
				*/
				
				//printf("5S ID:%d (%d,%d)\r\n",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
				//printf("OPEN LIST SIZE: %d\r\n",OPEN_LIST.size());
				if(OPEN_LIST.size() > 0)
				{
					for(int l = 0; l < OPEN_LIST.size();l++)
					{
						//printf("S ID:%d (%d,%d)\r\n",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
						//printf("Checking: OPEN ID: %d w/ SUCCESSOR ID: %d\r\n",OPEN_LIST.at(l).ID,SUCCESSORS.at(k).ID);
						if(OPEN_LIST.at(l).ID == SUCCESSORS.at(k).ID)
						{
							//if(OPEN_LIST.at(l).f < SUCCESSORS.at(k).f) { add_successor = 0; }
							add_successor = 0;
						}
					}
				}
				//printf("S ID:%d (%d,%d)\r\n",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
				for(int l = 0; l < CLOSED_LIST.size();l++)
				{
					//printf("S ID:%d (%d,%d)\r\n",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
					if(CLOSED_LIST.at(l).ID == SUCCESSORS.at(k).ID)
					{
						//if(CLOSED_LIST.at(l).f < SUCCESSORS.at(k).f) { add_successor = 0; }
						add_successor = 0;
						//break;
					}
				}
				if(add_successor == 1)
				{
					//printf("OPEN_LIST PUSH S ID:%d (%d,%d)\r\n",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
					OPEN_LIST.push_back(SUCCESSORS.at(k)); 
				}
				else
				{
					//printf("NOT PUSHING TO OPEN LIST: %d(%d,%d)\r\n",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
				}
			}
			CLOSED_LIST.push_back(q);
			for(int k = 0; k < OPEN_LIST.size();k++)
			{
			
			//printf("1S ID:%d (%d,%d)\r\n",SUCCESSORS.at(k).ID,SUCCESSORS.at(k).i,SUCCESSORS.at(k).j);
				if ((OPEN_LIST.at(k).i == Goal_i) and (OPEN_LIST.at(k).j == Goal_j))
				{
					//printf("PATH FOUND!\r\n");
					
					path_found =1;
					int path_complete = 0;
					int id = OPEN_LIST.at(k).ID;
					
					//printf("PATH ID's: ");
					//printf("%d(%d)->",id,OPEN_LIST.at(k).Parent_ID);
					id = OPEN_LIST.at(k).Parent_ID;
					while(path_complete == 0)
					{
						for(int l = 0; l < CLOSED_LIST.size();l++)
						{
							//printf("id: %d checking: %d\r\n",id,CLOSED_LIST.at(l).ID);
							if(id < 0)
							{
								path_complete = 1;
								//printf("PATH COMPLETE\r\n");
								//printf("Mapping_Node Path Search Returning\r\n");
								return 1;
							}
							else if(id == CLOSED_LIST.at(l).ID)
							{	
								//printf("%d(%d)->",id,CLOSED_LIST.at(l).Parent_ID);
								id = CLOSED_LIST.at(l).Parent_ID;
								PATH.push_back(CLOSED_LIST.at(l));
							}
						}
					}
					
				}
			}
			//printf("Searching...\r\n");
			iteration_counter++;
			
		}
		if(path_found == 0)
		{
			//printf("PATH NOT FOUND!\r\n");	
			return 0;
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
	printf("Mapping_Node Path Search Finished\r\n");
}
  /*ICARUS_Diagnostic.header.stamp = ros::Time::now();
  ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
  ICARUS_Diagnostic.Level = SEVERE;
  ICARUS_Diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
  ICARUS_Diagnostic.Description = "Node Closed.";
  Pub_ICARUS_Mapping_Diagnostic.publish(ICARUS_Diagnostic);*/

