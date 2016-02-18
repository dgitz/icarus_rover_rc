//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
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


int Pose_Valid = 0; 

//Motion Variables
int Steer_Command = STEER_CENTER;
int Drive_Command = DRIVE_NEUTRAL;
int armed_state = DISARMED;
int joystick_timeout_counter = 0;
int current_gear = GEAR_PARK;
int Joystick_Steer_Axis = 0;
int Joystick_Throttle_Axis = 3;
int Joystick_Disarm_Button = 14;
int Joystick_Arm_Button = 12;
int Reset_Position_Button = 3;
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
int Reset_Position = 0;
float current_speed = 0.0;
int current_speed_command = 0;
float current_Heading_deg = 0.0;
int Position_Initialized = 0;
//Other Variables

int DEBUG_MODE= 0;
int LOGGING_ENABLED = 0;
double dtime = 0.0;
string Operation_Mode = "";
geometry_msgs::Pose2D GoalPose;
sensor_msgs::Joy RoverCommand;
geometry_msgs::Pose2D TargetPose;
int Simulation_Initialized = 0;
int Rover_Status = STATUS_UNKNOWN;
ros::Publisher Pub_ICARUS_Motion_Controller_Diagnostic;

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
	printf("N: %f E: %f Heading: %f Bearing: %f Tx %f Ty: %f dx: %f dy: %f\r\n",current_Northing_m,current_Easting_m,current_Heading_deg,Target_Bearing_deg,TargetPose.x,TargetPose.y,dx,dy);
	//printf("Tx:%f,%f\r\n",TargetPose.x,msg->poses[Path_LookAhead].pose.position.x);
	

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

void ICARUS_Rover_GPS_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

	if(msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
	{ 
		Pose_Valid = 0; 
		//printf("GPS Status: NO FIX.\r\n");
	}
	else
	{
		Pose_Valid = 1;
		
		std::string zone;
		
		current_latitude = msg->latitude;
		current_longitude = msg->longitude;
		double temp_N_m;
		double temp_E_m;
		LLtoUTM(msg->latitude,msg->longitude,temp_N_m,temp_E_m,zone); 
		/*if(Position_Initialized == 0)
		{
			//origin_Northing_m = temp_N_m;
			//origin_Easting_m = temp_E_m;
			current_Northing_m = 0.0;
			current_Easting_m = 0.0;
			if((fabs(origin_Northing_m) > 0.0) && (fabs(origin_Easting_m) > 0.0))
			{
				Position_Initialized = 1;
			}
				
			
		}*/
		if((Position_Initialized == 1) && (current_gear != GEAR_PARK))
		{
               if(Reset_Position == 1)
               {
                    temp_N_m = origin_Northing_m;
                    temp_E_m = origin_Easting_m;               
               }
			current_Northing_m = temp_N_m-origin_Northing_m;
			current_Easting_m = temp_E_m-origin_Easting_m;
		}
	//printf("oN: %f cN: %f N: %f oE: %f cE: %f E: %f\r\n",origin_Northing_m,temp_N_m,current_Northing_m,origin_Easting_m,temp_E_m,current_Easting_m);
	}
	
}
void ICARUS_Rover_VFRHUD_Callback(const icarus_rover_rc::VFR_HUD::ConstPtr& msg)
{
	current_Heading_deg = (float)msg->heading;
	current_speed = msg->groundspeed;
}
void ICARUS_SimRover_Pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	//printf("x: %f y: %f head: %f\r\n",msg->x,msg->y,msg->theta);
    Pose_Valid = 1;
	current_Easting_m = msg->x;
	current_Northing_m = msg->y;
	current_Heading_deg = msg->theta*180.0/PI;

}
void ICARUS_Rover_State_Callback(const std_msgs::Int32::ConstPtr& msg)
{
	Rover_Status = msg->data;
}
void ICARUS_Rover_ManualControl_Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	

  float deadband = 0.1;
  joystick_timeout_counter = 0; //Reset Joystick Timeout Counter
  
  if(joy->buttons[Joystick_Disarm_Button] == 1) { armed_state = DISARMED; }
  else if(joy->buttons[Joystick_Arm_Button] == 1) { armed_state = ARMED; }
    if(Operation_Mode == "MANUAL")
    {
        float steer_axis_value = joy->axes[Joystick_Steer_Axis];
        float drive_axis_value = joy->axes[Joystick_Throttle_Axis];
        if(joy->buttons[Reset_Position_Button] == 1) { Reset_Position = 1; }
        else{ Reset_Position = 0; }
        if(armed_state == ARMED)
        {
            if(steer_axis_value > deadband) // Turning Left
            {
                Steer_Command = STEER_CENTER + steer_axis_value * (STEER_CENTER - STEER_MAX_RIGHT);
            }
            else if(steer_axis_value < (-1.0*deadband)) //Turning Right
            {
                Steer_Command = STEER_CENTER + steer_axis_value * (STEER_MAX_LEFT - STEER_CENTER);
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
	if(armed_state == ARMED)
	{
		ICARUS_Diagnostic.Diagnostic_Type = REMOTE_CONTROL;
		ICARUS_Diagnostic.Level = INFORMATION;
		ICARUS_Diagnostic.Diagnostic_Message = ROVER_ARMED;
		ICARUS_Diagnostic.Description = "Rover Armed";
		Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
	}
	else if(armed_state == DISARMED)
	{
		ICARUS_Diagnostic.Diagnostic_Type = REMOTE_CONTROL;
		ICARUS_Diagnostic.Level = INFORMATION;
		ICARUS_Diagnostic.Diagnostic_Message = ROVER_DISARMED;
		ICARUS_Diagnostic.Description = "Rover Disarmed";
		Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
	}
    

  
  
}
void ICARUS_Rover_AutoControl_Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	

  
    if(Operation_Mode == "HYBRID")
    {
		float deadband = 0.01;
        float steer_axis_value = joy->axes[Joystick_Steer_Axis];
        float drive_axis_value = joy->axes[Joystick_Throttle_Axis];
        if(joy->buttons[Reset_Position_Button] == 1) { Reset_Position = 1; }
        else{ Reset_Position = 0; }
        if(armed_state == ARMED)
        {
            if(steer_axis_value > deadband) // Turning Left
            {
                Steer_Command = STEER_CENTER + steer_axis_value * (STEER_CENTER - STEER_MAX_RIGHT);
            }
            else if(steer_axis_value < (-1.0*deadband)) //Turning Right
            {
                Steer_Command = STEER_CENTER + steer_axis_value *  (STEER_MAX_LEFT - STEER_CENTER);
            }
            else
            {
                Steer_Command = STEER_CENTER;
            }
            if(drive_axis_value > deadband) //Moving Forward
            {
                current_gear = GEAR_FORWARD;
                Drive_Command = DRIVE_NEUTRAL + 0.5*drive_axis_value * (DRIVE_MAX_FORWARD - DRIVE_NEUTRAL);
            }
            else if(drive_axis_value < (-1.0*deadband)) //Moving Reverse
            {
                Drive_Command = DRIVE_NEUTRAL + 0.5*drive_axis_value * (DRIVE_NEUTRAL - DRIVE_MAX_REVERSE);
                current_gear = GEAR_REVERSE;
            }
            else
            {
                Drive_Command = DRIVE_NEUTRAL;
                current_gear = GEAR_PARK;
            }
			
        }
    }
	/*ICARUS_Diagnostic.Diagnostic_Type = REMOTE_CONTROL;
	ICARUS_Diagnostic.Level = INFORMATION;
	ICARUS_Diagnostic.Diagnostic_Message = NO_ERROR;
	ICARUS_Diagnostic.Description = "Joystick Callback Received";*/
	
  
  
  
}
int main(int argc, char **argv)
{
	
	
	
	int INITIALIZED = 1;
	ros::init(argc, argv, "Motion_Controller");
	ros::NodeHandle nh("~");
	
	ICARUS_Diagnostic.System = ROVER;
	ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
	ICARUS_Diagnostic.Component = MOTION_CONTROLLER_NODE;
	ICARUS_Diagnostic.Diagnostic_Type = NO_ERROR;
	ICARUS_Diagnostic.Level = INFORMATION;
	ICARUS_Diagnostic.Diagnostic_Message = INITIALIZING;
	
	//nh.getParam("target_count",target_count);
	nh.getParam("DEBUG_MODE",DEBUG_MODE);
	nh.getParam("LOGGING_ENABLED",LOGGING_ENABLED);
	nh.getParam("Operation_Mode",Operation_Mode); //Should be: SIM, MANUAL, HYBRID, AUTO

	DEBUG_MODE = DEBUG;
	printf("MC: Op Mode: %s",Operation_Mode.c_str());
	time_t rawtime;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	char filename[80];
	strcpy(filename,ctime(&rawtime));
	strcat(filename,"MotionControllerNode.csv");
	ofstream out;
	if(LOGGING_ENABLED == 1) 
	{
		ofstream out(filename);
		out << "Time,GPS Valid,Armed,Gear,Drive_Command,Steer_Command,current_speed,current_latitude,current_longitude,current_heading,Northing,Easting" << endl;
	}
	
	ros::Subscriber Sub_Rover_ManualControl;
    ros::Subscriber Sub_Rover_AutoControl;
	ros::Subscriber GPS_State;
	ros::Subscriber VFRHUD_State;
	ros::Publisher Pub_Rover_RC;
	ros::Publisher Pub_Rover_Pose;
	ros::Subscriber Sub_Rover_Pose;
	ros::Subscriber Sub_Rover_Goal;
	//ros::Publisher Pub_Rover_Command;
	ros::Subscriber Sub_Rover_GlobalPath;
	ros::Publisher Pub_Rover_TraversedPath;
	ros::Subscriber Sub_Rover_State;
	////Pub_Rover_GlobalPath = nh.advertise<nav_msgs::Path>("ICARUS_Rover_GlobalPath",1000);
	if(Operation_Mode == "MANUAL")
	{

		Pub_ICARUS_Motion_Controller_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Motion_Controller_Diagnostic",1000);
		Sub_Rover_ManualControl = nh.subscribe<sensor_msgs::Joy>("/joy",1000,ICARUS_Rover_ManualControl_Callback);
		GPS_State = nh.subscribe<sensor_msgs::NavSatFix>("/Mavlink_Node/gps",1000,ICARUS_Rover_GPS_Callback); 
		VFRHUD_State = nh.subscribe<icarus_rover_rc::VFR_HUD>("/Mavlink_Node/vfr_hud",1000,ICARUS_Rover_VFRHUD_Callback);
		Pub_Rover_RC = nh.advertise<icarus_rover_rc::RC>("send_rc",1000);
		Pub_Rover_Pose = nh.advertise<geometry_msgs::Pose2D>("/Motion_Controller_Node/ICARUS_Rover_Pose",1000);
		nh.getParam("Origin_Easting_m",origin_Easting_m);
		nh.getParam("Origin_Northing_m",origin_Northing_m);
		Position_Initialized = 1;
		
	}
	else if(Operation_Mode == "SIM")
	{

		Pub_ICARUS_Motion_Controller_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Motion_Controller_Diagnostic",1000);
		Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Pose",1000,ICARUS_SimRover_Pose_Callback);
		Sub_Rover_Goal = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Goal",1000,ICARUS_Rover_Goal_Callback);
		origin_Easting_m = 0.0;
		origin_Northing_m = 0.0;
		Position_Initialized = 1;
		
	}
	else if(Operation_Mode == "HYBRID")
	{
		Pub_ICARUS_Motion_Controller_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Motion_Controller_Diagnostic",1000);
		Sub_Rover_ManualControl = nh.subscribe<sensor_msgs::Joy>("/joy",1000,ICARUS_Rover_ManualControl_Callback);
        Sub_Rover_AutoControl = nh.subscribe<sensor_msgs::Joy>("/Navigation_Node/ICARUS_Rover_Command",1000,ICARUS_Rover_AutoControl_Callback);
		//GPS_State = nh.subscribe<sensor_msgs::NavSatFix>("/Mavlink_Node/gps",1000,ICARUS_Rover_GPS_Callback); 
		//VFRHUD_State = nh.subscribe<icarus_rover_rc::VFR_HUD>("/Mavlink_Node/vfr_hud",1000,ICARUS_Rover_VFRHUD_Callback);
		Pub_Rover_RC = nh.advertise<icarus_rover_rc::RC>("send_rc",1000);
		Sub_Rover_Pose = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Pose",1000,ICARUS_SimRover_Pose_Callback);
		Sub_Rover_Goal = nh.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Goal",1000,ICARUS_Rover_Goal_Callback);
		origin_Easting_m = 0.0;
		origin_Northing_m = 0.0;
		Position_Initialized = 1;
	}
	Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
	Sub_Rover_State = nh.subscribe<std_msgs::Int32>("/Robot_Controller_Node/ICARUS_State",1000,ICARUS_Rover_State_Callback);
	Pub_Rover_TraversedPath = nh.advertise<nav_msgs::Path>("ICARUS_Rover_TraversedPath",1000);
	//ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformBroadcaster base_broadcaster;
	//ros::Publisher Pub_ICARUS_Rover_Pose = nh.advertise<geometry_msgs::Pose2D>("ICARUS_Rover_Pose",1000);

	ros::Rate loop_rate(50);
	std::clock_t    start;
	
	::icarus_rover_rc::RC RC_Command;
	int direction = 1;
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	double second_timer = 0.0;
	struct timeval start_timer,current_timer;
	long  seconds, useconds;
	gettimeofday(&current_timer,NULL);
	gettimeofday(&start_timer,NULL);
	nav_msgs::Path TraversedPath;
	TraversedPath.header.stamp = ros::Time::now();
	TraversedPath.header.frame_id = "map";
	TraversedPath.poses.resize(120);
	int Traversed_Path_Points = 0;
	while( ros::ok() && INITIALIZED)
	{
		start = std::clock();
	  	ros::spinOnce();
		last_time = current_time;
		current_time = ros::Time::now();
	  	loop_rate.sleep();
		gettimeofday(&current_timer,NULL);
		seconds = current_timer.tv_sec - start_timer.tv_sec;
		useconds = current_timer.tv_usec - start_timer.tv_usec;
		second_timer += seconds + useconds/1000000.0;
		
		gettimeofday(&start_timer,NULL);
		if(second_timer > 1.0)
		{
			second_timer = 0.0;
			if(Pose_Valid == 1)
			{
				
				if(Traversed_Path_Points < 120)
				{
					
					TraversedPath.poses[Traversed_Path_Points].header.stamp = ros::Time::now();
					TraversedPath.poses[Traversed_Path_Points].pose.position.x = current_Easting_m;
					TraversedPath.poses[Traversed_Path_Points].pose.position.y = current_Northing_m;
					Traversed_Path_Points++;
				}
				if(Traversed_Path_Points >= 120) 
				{	
					geometry_msgs::PoseStamped newpoint;
					newpoint.header.stamp = ros::Time::now();
					newpoint.pose.position.x = current_Easting_m;
					newpoint.pose.position.y = current_Northing_m;
					TraversedPath.poses.erase(TraversedPath.poses.begin());
					TraversedPath.poses.push_back(newpoint);
					
					
				}
				
				
				
			}
			
		}
		TraversedPath.header.stamp = ros::Time::now();
		Pub_Rover_TraversedPath.publish(TraversedPath);
		
		if((Operation_Mode == "MANUAL") || (Operation_Mode=="HYBRID"))
		{
			joystick_timeout_counter++;  //Increment Joystick Timeout Counter
			if(joystick_timeout_counter > 100) 
			{ 
				//printf("Joystick Callback Timeout\r\n");// }
				armed_state = DISARMED; 
				ICARUS_Diagnostic.Diagnostic_Type = REMOTE_CONTROL;
				ICARUS_Diagnostic.Level = CAUTION;
				ICARUS_Diagnostic.Diagnostic_Message = DROPPING_PACKETS;
				ICARUS_Diagnostic.Description = "Joystick Callack Timeout";
				Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
				
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
				/*if((DEBUG_MODE == DEBUG) and (Pose_Valid == 0)) 
				{ 
					Pose_Valid = 1; 
					current_Northing_m = 0.0;//current_Northing_m + .01;
					current_Easting_m = 0.0;//current_Easting_m + .01;//0.0;
					current_Heading_deg = 0.0;
				} //DEBUGGING ONLY*/
				if(Pose_Valid == 1)
				{
					//cout << "Gear: " << current_gear << " Thr: " << Drive_Command << " Sp: " << current_speed << " GPS: " << Pose_Valid << " Lat: " << current_latitude << " Long: " << current_longitude << " Head: " << current_heading << endl;
					//
					struct timeval mytime;
					gettimeofday(&mytime,NULL);
					double cur_time = (double)(mytime.tv_sec + mytime.tv_usec/1000000.0);
					
					if(LOGGING_ENABLED == 1)
					{
						out << setprecision(16) << cur_time << "," << Pose_Valid << "," << armed_state << "," << current_gear << "," << Drive_Command << "," << Steer_Command << "," << current_speed << "," << current_latitude << "," << current_longitude << "," << current_Heading_deg << "," << current_Northing_m << "," << current_Easting_m <<  endl;
					}
					//printf("T: %f Armed: %d Gear: %d Thr: %d St: %d Sp: %f GPS: %d Head: %f N: %f E: %f\r\n",cur_time,armed_state,current_gear,Drive_Command,Steer_Command,current_speed,Pose_Valid,current_Heading_deg,current_Northing_m,current_Easting_m);
					geometry_msgs::Pose2D cur_Pose;
					cur_Pose.x = current_Easting_m;
					cur_Pose.y = current_Northing_m;
					cur_Pose.theta = current_Heading_deg*PI/180.0;
					Pub_Rover_Pose.publish(cur_Pose);
					
					
				}
				else
				{
					
				}
				//for(int i = 0; i < 8; i++) { RC_Command.channel[i] = Steer_Command; }
				
				Pub_Rover_RC.publish(RC_Command);
				
				if(Pose_Valid == 0)
				{
					ICARUS_Diagnostic.Diagnostic_Type = SENSORS;
					ICARUS_Diagnostic.Level = CAUTION;
					ICARUS_Diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
					ICARUS_Diagnostic.Description = "GPS Position Not Available";
					Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
				}
				else if(Pose_Valid == 1)
				{
					ICARUS_Diagnostic.Diagnostic_Type = SENSORS;
					ICARUS_Diagnostic.Level = INFORMATION;
					ICARUS_Diagnostic.Diagnostic_Message = NO_ERROR;
					ICARUS_Diagnostic.Description = "GPS Position Available";
					Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
				}
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
		else if(Operation_Mode == "SIM")
		{

		}
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "/map";
		odom_trans.child_frame_id = "/odom";
		odom_trans.transform.translation.x = 0.0;
		odom_trans.transform.translation.y = 0.0;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		odom_broadcaster.sendTransform(odom_trans);
		geometry_msgs::Quaternion base_quat = tf::createQuaternionMsgFromYaw(-1.0*current_Heading_deg*PI/180.0);
		geometry_msgs::TransformStamped base_trans;
		base_trans.header.stamp = current_time;
		base_trans.header.frame_id = "/odom";
		base_trans.child_frame_id = "/base_link";
		base_trans.transform.translation.x = current_Easting_m;
		base_trans.transform.translation.y = current_Northing_m;
		base_trans.transform.translation.z = -0.2794;
		base_trans.transform.rotation = base_quat;
		base_broadcaster.sendTransform(base_trans);
		//printf("Published base_link -> odom tf\r\n");
		

		
		/*nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = current_Easting_m;
		odom.pose.pose.position.y = current_Northing_m;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = current_speed*sin(0.0);
		odom.twist.twist.linear.y = current_speed*cos(0.0);
		odom.twist.twist.angular.z = 0.0;

		//publish the message
		odom_pub.publish(odom);*/


		dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
	    ICARUS_Diagnostic.Diagnostic_Type = NO_ERROR;
		ICARUS_Diagnostic.Level = INFORMATION;
		ICARUS_Diagnostic.Diagnostic_Message = NO_ERROR;
		ICARUS_Diagnostic.Description = "MC Node Operational";
		Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
		
		
		
	}
	if(LOGGING_ENABLED == 1)
	{
		out.close();
	}
	/*
	ICARUS_Diagnostic.header.stamp = ros::Time::now();
	ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
	ICARUS_Diagnostic.Level = SEVERE;
	ICARUS_Diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
	ICARUS_Diagnostic.Description = "Node Closed.";
	Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
  */
}
