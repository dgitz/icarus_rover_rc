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
#include <std_msgs/Int32.h>
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
#define PI 3.14159265359


#include <sys/time.h>


using namespace std;

int DEBUG_MODE= 0;
int LOGGING_ENABLED = 0;
double dtime = 0.0;
string Operation_Mode = "";
geometry_msgs::Pose2D GoalPose;
int Rover_Status = STATUS_STOPPED;
int Simulation_Status = STATUS_UNKNOWN;
int Load_Gene_List = 1;
int Simulation_Initialized = 0;
double current_Northing_m = 0.0;
double current_Easting_m = 0.0;
double Goal_Northing_m = 0.0;
double Goal_Easting_m = 0.0;
float current_Heading_deg = 0.0;
double Distance_Threshold_m = 0.0;
double Time_Threshold_sec = 0.0;
double Time_On_Mission = 0.0;
int Goal_Reached = 0;

std_msgs::Int32 ROSRover_State;
::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
::icarus_rover_rc::ICARUS_GeneList GeneList;
int debug_publish = 1;

void ICARUS_SimulationState_Callback(const std_msgs::Int32::ConstPtr& msg)
{
	try
	{
		Simulation_Status = msg->data;
		if((Simulation_Status == STATUS_GENELIST_READY) || (debug_publish == 1))
		{
			debug_publish = 0;
			std::vector<std::string> gene_names;
			std::vector<float> gene_values;
			std::string gene_name;
			float value;
			//printf("I should read the Gene List.\r\n");
			FILE * fp;
			fp = fopen("/home/linaro/catkin_ws/src/icarus_rover_rc/gene_files/GeneList.txt","r");
			int stage = 0;
			fscanf(fp,"Stage:%d\r\n",&stage);
			int skip_line = 1;
			while(! feof (fp))
			{
				fscanf(fp,"%s %f\r\n",gene_name.c_str(),&value);
				std::string temp = gene_name.c_str();
				//printf("G: %s V: %lf\r\n",gene_name.c_str(),value);
				gene_names.push_back(temp);
				gene_values.push_back(value);

			}
			//gene_names.push_back("a");
			//gene_names.push_back("b");
			//gene_names.push_back("c");
			fclose(fp);
			GeneList.Stage = stage;
			GeneList.GeneName.resize(gene_names.size());
			GeneList.GeneValue.resize(gene_names.size());
			for(int i = 0; i < gene_names.size();i++)
			{
				//printf("G: %s V: %lf\r\n",gene_names.at(i).c_str(),gene_values.at(i));
				std::string tempstr = gene_names.at(i).c_str();
				GeneList.GeneName[i] = tempstr;
				GeneList.GeneValue[i] = gene_values.at(i);
			}
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
	}
	
	
}
void ICARUS_SimRover_Pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	try
	{
		//printf("x: %f y: %f head: %f\r\n",msg->x,msg->y,msg->theta);
		current_Easting_m = msg->x;
		current_Northing_m = msg->y;
		current_Heading_deg = msg->theta*180.0/PI;
		double dx = fabs(current_Easting_m - GoalPose.x);
		double dy = fabs(current_Northing_m - GoalPose.y);
		//printf("Goal E: %f E: %f Goal N: %f N: %f dx: %f dy: %f\r\n",GoalPose.x,current_Easting_m,GoalPose.y,current_Northing_m,dx,dy);
		if((dx < Distance_Threshold_m) and (dy < Distance_Threshold_m) and (Simulation_Initialized == 1))
		{
			//printf("Robot Controller: GOAL REACHED!\r\n");
			Goal_Reached = 1;
		}
		else
		{
			Goal_Reached = 0;
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
	}
	
}
void ICARUS_Rover_Goal_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	try
	{
		Simulation_Initialized = 1;
		GoalPose.x = msg->x;
		GoalPose.y = msg->y;
		GoalPose.theta = msg->theta;
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
	}
	//printf("Got a Goal x: %f y: %f theta: %f\r\n",msg->x,msg->y,msg->theta);
	
}
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
	nh.getParam("Distance_Threshold_m",Distance_Threshold_m);
	nh.getParam("Time_Threshold_sec",Time_Threshold_sec);
	nh.getParam("Goal_Easting_m",Goal_Easting_m);
	nh.getParam("Goal_Northing_m",Goal_Northing_m);
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
	ros::Publisher Pub_Rover_State;
	ros::Subscriber Sub_Simulation_State;
	ros::Subscriber Sub_Rover_Pose;
	ros::Subscriber Sub_Rover_Goal;
	ros::Publisher Pub_Rover_Goal;
	if(Operation_Mode == "LIVE")
	{
		Pub_ICARUS_Robot_Controller_Node_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Robot_Controller_Node_Diagnostic",1000);
		Pub_Rover_State = nh.advertise<std_msgs::Int32>("ICARUS_State",1000);
		Pub_Rover_Goal = nh.advertise<geometry_msgs::Pose2D>("Rover_Goal",1000);
		GoalPose.x = Goal_Easting_m;
		GoalPose.y = Goal_Northing_m;
		GoalPose.theta = 0.0;
		
	}
	else if(Operation_Mode == "SIM")
	{
		Pub_ICARUS_Robot_Controller_Node_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Robot_Controller_Node_Diagnostic",1000);

		Pub_ICARUS_GeneList = nh.advertise<icarus_rover_rc::ICARUS_GeneList>("ICARUS_GeneList",1000);
		Pub_Rover_State = nh.advertise<std_msgs::Int32>("ICARUS_State",1000);
		Sub_Simulation_State = nh.subscribe<std_msgs::Int32>("/Matlab_Node/ICARUS_State",1000,ICARUS_SimulationState_Callback);
		Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Pose",1000,ICARUS_SimRover_Pose_Callback);
		Sub_Rover_Goal = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Goal",1000,ICARUS_Rover_Goal_Callback);

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
				Pub_Rover_Goal.publish(GoalPose);
			}		
			catch(exception& e)
			{
				cout << "RC Node: " << e.what() << endl;
			}
		
		}
		else if(Operation_Mode == "SIM")
		{
			try
			{
				//printf("Robot Controller: Sim State: %d Rover State: %d\r\n",Simulation_Status,Rover_Status);
				//Simulation_Status = STATUS_GENELIST_READY; //DEBUG ONLY
				//if(((Simulation_Status == STATUS_GENELIST_READY) || (Simulation_Status == STATUS_AUTONOMOUS_CONTROL)) && (Load_Gene_List == 1)) //Load Gene List
				if(((Simulation_Status == STATUS_AUTONOMOUS_CONTROL) || (Simulation_Status == STATUS_GENELIST_READY)) && (Load_Gene_List == 1)) //Load Gene List
				{
					
					//Load Gene List Here
					Load_Gene_List = 0;
					for(int i = 0; i < 10; i++)//Publish the GeneList a bunch of times, but don't do it anymore than necessary.
					{
						int GeneCount = 10;
						//GeneList.GeneName.resize(GeneCount);
						//GeneList.GeneValue.resize(GeneCount);
						Pub_ICARUS_GeneList.publish(GeneList);
					}
					Goal_Reached = 0;
					
					Time_On_Mission = 0.0;
					Rover_Status = STATUS_NAVIGATING;
				}
				if( Rover_Status == STATUS_GOAL_REACHED)
				{
					Rover_Status = STATUS_STOPPED;
				}
				else if(Rover_Status == STATUS_MAXTIME_REACHED)
				{
					Rover_Status = STATUS_STOPPED;
				}
				else if( Rover_Status == STATUS_NAVIGATING)
				{
					Load_Gene_List = 1;
					//Arm Code Here
					if(Goal_Reached == 1)
					{
						Rover_Status = STATUS_GOAL_REACHED;
					}
					else if(Time_On_Mission > Time_Threshold_sec)
					{
						Rover_Status = STATUS_MAXTIME_REACHED;
						printf("Time On Mission to long!\r\n");
					}

				}
				Time_On_Mission = dtime + Time_On_Mission;
				//printf("Time On Mission: %f\r\n",Time_On_Mission);
				//Rover_Status = STATUS_NAVIGATING; //DEBUG ONLY
				ROSRover_State.data = Rover_Status;
				Pub_Rover_State.publish(ROSRover_State);
			}
			catch(exception& e)
			{
				cout << "RC Node: " << e.what() << endl;
				ROS_ERROR("%s",e.what());
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
