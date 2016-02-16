//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
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
double Desired_Steering = 0.0;
double Steer_Target = 0.0; //Steering based on Genes and Target 
double Steer_Obstacle = 0.0; //Steering based on Genes and Obstacle
double Throttle_Target = 0.0;
double Throttle_Obstacle = 0.0;
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
double distance_to_Goal = 0.0;
float current_speed = 0.0;
double ref_pose_East = current_Easting_m;
double ref_pose_North = origin_Northing_m;
int ref_pose_visits = 0;
int current_speed_command = 0;
float current_Heading_deg = 0.0;
int Rover_Status = STATUS_UNKNOWN;

//EA Variables
int EA_Stage = 0;
float Gene_Steering_Target = 0.0;
float Gene_Throttle_Target = 0.0;
float Gene_Path_LookAhead = 0.0;
std::vector<float> Gene_SonarBeam_SteerAngle;
std::vector<float> Gene_SonarBeam_Throttle;
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
int stuck_timer = 0;
int Forward_Neutral_Reverse = 0;


::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
double constrainAngle(double x){
	try
	{
		x = fmod(x + 180.0,360.0);
		if (x < 0.0)
			x += 360.0;
		return x - 180.0;
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
}
void GlobalPath_Callback(const nav_msgs::Path::ConstPtr& msg)
{
	try
	{
		int PathSize = msg->poses.size();
		//float Gene_Path_LookAhead = .25;
		int path_index = Gene_Path_LookAhead*PathSize;
		if(path_index < 0){path_index = 0;}
		else if(path_index > PathSize) { path_index = PathSize-1; }
	
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
		
		//printf("Tx:%f,%f\r\n",TargetPose.x,msg->poses[Path_LookAhead].pose.position.x);
		/*if(Goal_Reached == 1)
		{
			Desired_Throttle = 0.0;
		}
		else
		{
			Desired_Throttle = Gene_Throttle_Target;
		}*/
	
		float delta_target_angle = Target_Bearing_deg - current_Heading_deg;
		delta_target_angle = constrainAngle(delta_target_angle);

		if(ref_pose_visits < 100) //Moving along fine
		{
			Steer_Target = Gene_Steering_Target*(delta_target_angle)/180.0;
		}
		
		//printf("N: %f E: %f Heading: %f Bearing: %f Tx %f Ty: %f TTh: %f St: %f\r\n",current_Northing_m,current_Easting_m,current_Heading_deg,Target_Bearing_deg,TargetPose.x,TargetPose.y,delta_target_angle,Steer_Target);
		//printf("Delta Target Angle: %f Steer: %f\r\n",delta_target_angle,Steer_Target);
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
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
		ROS_ERROR("%s",e.what());
	}
	//printf("Got a Goal x: %f y: %f theta: %f\r\n",msg->x,msg->y,msg->theta);
	
}

void GeneList_Callback(const icarus_rover_rc::ICARUS_GeneList::ConstPtr& msg)
{
	try
	{
		//printf("Nav Node: Got a new Gene List!\r\n");
		EA_Stage = msg->Stage;
		std::vector<std::string> gene_names = msg->GeneName;
		std::vector<float> gene_values = msg->GeneValue;
		
		for(int i = 0; i < gene_names.size();i++)
		{
			std::string sonarbeam_steerangle = "Weight_SonarBeam_SteerAngle_";
			std::string sonarbeam_throttle = "Weight_SonarBeam_Throttle_";
			std::size_t gene_steer_found = gene_names.at(i).find(sonarbeam_steerangle);
			std::size_t gene_throttle_found = gene_names.at(i).find(sonarbeam_throttle);
			//printf("Gene Name: %s\r\n",gene_names.at(i).c_str());
			if(gene_names.at(i).compare("Weight_SteerAngle_To_Target") == 0)
			{
				Gene_Steering_Target = gene_values.at(i);
				//printf("Got Gene Steering Target: %lf\r\n",Gene_Steering_Target);
			}
			else if(gene_names.at(i).compare("Weight_Throttle_To_Target") == 0)
			{
				Gene_Throttle_Target = gene_values.at(i);
				if(Gene_Throttle_Target >= 1.0){Gene_Throttle_Target = 1.0; }
				else if(Gene_Throttle_Target <= 0.1) { Gene_Throttle_Target = 0.1; }
				//printf("Got Gene Weight_Throttle_To_Target: %lf\r\n",Gene_Throttle_Target);
			}
			else if(gene_names.at(i).compare("Weight_Path_LookAhead") == 0)
			{
				Gene_Path_LookAhead = gene_values.at(i);
				if(Gene_Path_LookAhead >= 0.5){Gene_Path_LookAhead = 0.5; }
				else if(Gene_Path_LookAhead <= 0.1) {Gene_Path_LookAhead = 0.1; }
				//printf("Got Weight_Path_LookAhead: %lf\r\n",Gene_Path_LookAhead);
			}
			else if(gene_steer_found != std::string::npos)
			{
				std::string tempstr = gene_names.at(i).substr(sonarbeam_steerangle.length(),gene_names.at(i).length());
				int index = atoi(tempstr.c_str());
				Gene_SonarBeam_SteerAngle.at(index) = gene_values.at(i);
				//printf("Nav Node: Gene: %d %lf\r\n",index,gene_values.at(i));
				
			}
			else if(gene_throttle_found != std::string::npos)
			{
				std::string tempstr = gene_names.at(i).substr(sonarbeam_throttle.length(),gene_names.at(i).length());
				int index = atoi(tempstr.c_str());
				Gene_SonarBeam_Throttle.at(index) = gene_values.at(i);
			}
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
	
}

void ICARUS_Sonar_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//printf("Got a Scan!\r\n");
	try
	{
		double start_angle = msg->angle_min;
		double angle_increment = msg->angle_increment;
		double current_angle = start_angle;
		double temp1 = 0.0;
		double temp2 = 0.0;
		int beam_count = msg->ranges.size();
		//std::vector<double> ranges = msg->ranges;
		for(int i = 0; i < beam_count;i++)
		{
			
			if(msg->ranges[i] > 0)
			{
				if(i == 65) // Middle of front Beam.  Angle is 0, leads to discontinuities.
				{
					//temp = temp + (Gene_SonarBeam_SteerAngle.at(i)*msg->ranges[i])/(current_angle);
				}
				else
				{
					temp1 = temp1 + (Gene_SonarBeam_SteerAngle.at(i))/(current_angle*(180.0/PI)*msg->ranges[i]);
					if(msg->ranges[i] < 3.0)
					{
						temp2 = temp2 + -1.0*abs((Gene_SonarBeam_Throttle.at(i))/(current_angle*(180.0/PI)*msg->ranges[i]));
					}
					else if(msg->ranges[i] > 3.0)
					{
						temp2 = temp2 + 1.0*abs((Gene_SonarBeam_Throttle.at(i))/(current_angle*(180.0/PI)*msg->ranges[i]));
					}
					//printf("i: %d G: %f Th: %f D: %f Sum: %f\r\n",i,Gene_SonarBeam_SteerAngle.at(i),current_angle*(180.0/PI),msg->ranges[i],temp1);
					//printf("i: %d G: %f Th: %f D: %f Sum: %f\r\n",i,Gene_SonarBeam_Throttle.at(i),current_angle*(180.0/PI),msg->ranges[i],temp2);
				}
				//printf("i: %d Th: %f D: %f W: %f Sum: %f\r\n",i,current_angle,msg->ranges[i],Gene_SonarBeam_SteerAngle.at(i),temp1);
				//printf("i: %d Th: %f D: %f W: %f Sum: %f\r\n",i,current_angle,msg->ranges[i],Gene_SonarBeam_Throttle.at(i),temp2);
			}
			current_angle = current_angle + angle_increment;
			
		}
		Steer_Obstacle = -temp1/13.0;
		Throttle_Obstacle = temp2/50.0;
		//printf("So: %f\r\n",Steer_Obstacle);
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
}
void ICARUS_Rover_VFRHUD_Callback(const icarus_rover_rc::VFR_HUD::ConstPtr& msg)
{
	try
	{
		if(DEBUG_MODE == 0)
		{
			current_Heading_deg = (float)msg->heading;
			current_speed = msg->groundspeed;
		}
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
	}
}
void ICARUS_Rover_State_Callback(const std_msgs::Int32::ConstPtr& msg)
{
	try
	{
		Rover_Status = msg->data;
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
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

		//printf("NN: Goal E: %f E: %f Goal N: %f N: %f dx: %f dy: %f\r\n",GoalPose.x,current_Easting_m,GoalPose.y,current_Northing_m,dx,dy);
		if((dx < 0.5) and (dy < 0.5) and (Simulation_Initialized == 1))
		{
			//printf("GOAL REACHED!\r\n");
			Goal_Reached = 1;
		}
		else
		{
			Goal_Reached = 0;
		}
		dx = current_Easting_m - ref_pose_East;
		dy = current_Northing_m - ref_pose_North;
		double distance_ref = pow(pow(dx,2.0) + pow(dy,2.0),0.5);
		if((Rover_Status == STATUS_NAVIGATING) || (Rover_Status == STATUS_AUTONOMOUS_CONTROL))
		{
			if(distance_ref > 2.0)
			{
				ref_pose_East = current_Easting_m;
				ref_pose_North = current_Northing_m;
				ref_pose_visits = 0.0;
			}
			else if(distance_ref < 1.5)
			{
				ref_pose_visits++;
			}
			ROS_DEBUG("Visits: %d\r\n",ref_pose_visits);
		}
		else
		{
			ref_pose_East = current_Easting_m;
			ref_pose_North = current_Northing_m;
			ref_pose_visits = 0.0;
		}
		
		
		
	}
	catch(exception& e)
	{
		cout << "RC Node: " << e.what() << endl;
		ROS_ERROR("%s",e.what());
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
	ros::Subscriber Sub_Rover_State;
	RoverCommand.axes.resize(8);
	RoverCommand.buttons.resize(8);
	Gene_SonarBeam_SteerAngle.resize(130);
	Gene_SonarBeam_Throttle.resize(130);
	for(int i = 0; i < 130; i++)
	{
		Gene_SonarBeam_SteerAngle.at(i) = 0.0;
		Gene_SonarBeam_Throttle.at(i) = 0.0;
	}
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
    else if(Operation_Mode == "HYBRID")
    {
        Pub_ICARUS_Navigation_Node_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Motion_Controller_Diagnostic",1000);
		Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Pose",1000,ICARUS_SimRover_Pose_Callback);
		Sub_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Sonic_Controller_Node/ICARUS_Sonar_Scan",1000,ICARUS_Sonar_Scan_Callback);
		Sub_Rover_Goal = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Goal",1000,ICARUS_Rover_Goal_Callback);
		Sub_Rover_GlobalPath = nh.subscribe<nav_msgs::Path>("/Mapping_Node/ICARUS_Rover_GlobalPath",1000,GlobalPath_Callback);
		Sub_GeneList = nh.subscribe<icarus_rover_rc::ICARUS_GeneList>("/Robot_Controller_Node/ICARUS_GeneList",1000,GeneList_Callback);
        Pub_Rover_Command = nh.advertise<sensor_msgs::Joy>("ICARUS_Rover_Command",1000);
    }
	Pub_ICARUS_Navigation_Node_Diagnostic.publish(ICARUS_Diagnostic);
	Sub_Rover_State = nh.subscribe<std_msgs::Int32>("/Robot_Controller_Node/ICARUS_State",1000,ICARUS_Rover_State_Callback);
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
			catch(exception& e)
			{
				cout << "RC Node: " << e.what() << endl;
				ROS_ERROR("%s",e.what());
			}
		
		}
		else if((Operation_Mode == "SIM") || (Operation_Mode=="HYBRID"))
		{
			try
			{
				if((Rover_Status == STATUS_NAVIGATING) || (Rover_Status == STATUS_AUTONOMOUS_CONTROL))
				{
					Desired_Throttle = Gene_Throttle_Target + Throttle_Obstacle;
					
				}
				else
				{
					Desired_Throttle = 0.0;
				}
				if(Goal_Reached == 1)
				{ 
					//printf("NN: Goal Reached: %d\r\n",Goal_Reached);
					Desired_Throttle = 0.0; 
				}

				RoverCommand.header.stamp = ros::Time::now();
				
				Desired_Steering = -1.0*Steer_Target + Steer_Obstacle;
				
				if(Goal_Reached == 0)
				{
					if(Desired_Steering > 1.0) { Desired_Steering = 1.0; }
					else if(Desired_Steering < -1.0) { Desired_Steering = -1.0; }
					if(Desired_Throttle > 1.0) { Desired_Throttle = 1.0; }
					else if(Desired_Throttle < -1.0) { Desired_Throttle = -1.0; }
					if(Desired_Throttle > 0.0){ Forward_Neutral_Reverse = 1; }
					else if(Desired_Throttle < 0.0) { Forward_Neutral_Reverse = -1; }
					if(Desired_Throttle < 0.0) 
					{ 
						if(Desired_Steering < 0.0)
						{
							Desired_Steering = 0.5;
						}
						else
						{
							Desired_Steering = -0.5;
						}
					}
					if((abs(Desired_Throttle) < 0.05) and ((Rover_Status == STATUS_NAVIGATING) || (Rover_Status == STATUS_AUTONOMOUS_CONTROL)))
					{
						//printf("I'm stuck!\r\n");
						stuck_timer++;
					}
					if((abs(Desired_Throttle) > 0.5) and ((Rover_Status == STATUS_NAVIGATING) || (Rover_Status == STATUS_AUTONOMOUS_CONTROL)))
					{
						stuck_timer = 0;
					}
					double temp = Forward_Neutral_Reverse * stuck_timer * .05;
					Desired_Throttle = Desired_Throttle + temp;
				}
				printf("NN: Steer: T: %f O: %f P: %f\r\n",Steer_Target,Steer_Obstacle,Desired_Steering);
				printf("NN: Throttle: %lf,%lf,%lf\r\n",Desired_Throttle,Gene_Throttle_Target,Throttle_Obstacle);
				RoverCommand.axes[Joystick_Steer_Axis] = Desired_Steering;
				RoverCommand.axes[Joystick_Throttle_Axis] = Desired_Throttle;
				Pub_Rover_Command.publish(RoverCommand);
				
			}
			catch(exception& e)
			{
				cout << "RC Node: " << e.what() << endl;
				ROS_ERROR("%s",e.what());
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
