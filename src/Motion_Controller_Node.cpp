//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>


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
#include "icarus_rover_rc/ICARUS_Probe_Status.h"
#include "icarus_rover_rc/ICARUS_Probe_Command.h"
#include "icarus_rover_rc/ICARUS_Diagnostic.h"

#include "roscopter/State.h"


using namespace std;

int Extended_Switch = 0;
int Retracted_Switch = 0;
int Forward_Timeout = 0;
int Reverse_Timeout = 0;
int Recharge_Command = RECHARGE_WAIT;
double forward_timer = 0.0;
double reverse_timer = 0.0;

//Communication Variables
int token_index = 0;
string MC_Device = "";
int Baud_Rate = -1;
int in_message_started = 0;
int in_message_completed = 0;
int Gps_Valid = 0; 

//Motion Variables
int Steer_Command;
int Drive_Command;
int armed_state;



//Other Variables
int Probe_State = PROBE_RETRACTED;
int Probe_Error = NO_ERROR;
int DEBUG_MODE;
double dtime = 0.0;
int temp1;


void ICARUS_Probe_Command_Callback(const icarus_rover_rc::ICARUS_Probe_Command::ConstPtr& msg)  //Process ICARUS Probe Command Message
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  Recharge_Command = 0;//msg.Charge_Command;
  
}

void ICARUS_Rover_GPS_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	//printf("GPS Status: %d",msg->status.status);
	if(msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
	{ 
		Gps_Valid = 0; 
	}
	else
	{
		Gps_Valid = 1;
	}
}
void ICARUS_Rover_Control_Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//Assume Joy values are float, -1.0 to 1.0
  Steer_Command = (int)(joy->axes[0]+0.5)*255.0;
  Drive_Command = (int)(joy->axes[1]+0.5)*255.0;
  
}
void Recharge_FSM(double dtime)
{
  switch (Probe_State)
  {
    case PROBE_MOVING_FORWARD:
      forward_timer += dtime;
      if (Extended_Switch == true)
      { 
        Probe_State = PROBE_EXTENDED; 
        Probe_Error = NO_ERROR;
      }
      //servoWrite(WINCH_MOTOR_PIN, WINCH_MOTOR_FORWARD);
      break;
    case PROBE_MOVING_REVERSE:
      reverse_timer += dtime;
      if (Retracted_Switch == true) 
      { 
        Probe_State = PROBE_RETRACTED; 
      }
      //servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_REVERSE);
      break;
    case PROBE_EXTENDED:
      forward_timer = 0.0;
      reverse_timer = 0.0;
      if (Recharge_Command == RECHARGE_STOP)
      {
        Probe_State = PROBE_MOVING_REVERSE;
      }
      //servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
    case PROBE_RETRACTED:
      forward_timer = 0.0;
      reverse_timer = 0.0;
      if (Recharge_Command == RECHARGE_START) 
      { 
        Probe_State = PROBE_MOVING_FORWARD; 
        Probe_Error = NO_ERROR;
      }
      //servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
    default:
      //servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
  }
  if (forward_timer > FORWARD_TIME_LIMIT){ Probe_Error = EXTENSION_ERROR; }
  if (reverse_timer > REVERSE_TIME_LIMIT){ Probe_State = RETRACTION_ERROR; }
}
int main(int argc, char **argv)
{
	int INITIALIZED = 1;
	ros::init(argc, argv, "Motion_Controller");
	ros::NodeHandle nh("~");
	//nh.getParam("target_count",target_count);
	nh.getParam("DEBUG_MODE",DEBUG_MODE);
  
	

	ros::Subscriber Sub_ICARUS_Probe_Command_Callback = nh.subscribe("ICARUS_Probe_Command", 1000, ICARUS_Probe_Command_Callback);
	ros::Publisher Pub_ICARUS_Probe_Status = nh.advertise<icarus_rover_rc::ICARUS_Probe_Status>("ICARUS_Probe_Status", 1000);  
	ros::Publisher Pub_ICARUS_Motion_Controller_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Motion_Controller_Diagnostic",1000);
	ros::Subscriber Pub_Rover_Control = nh.subscribe<sensor_msgs::Joy>("ICARUS_Rover_Control",1000,ICARUS_Rover_Control_Callback);
	ros::Subscriber GPS_State = nh.subscribe<sensor_msgs::NavSatFix>("gps",1000,ICARUS_Rover_GPS_Callback);  //I only care if the GPS has a fix.  I don't care about the Pose.
	//ros::Publisher Pub_ICARUS_Rover_Pose = nh.advertise<geometry_msgs::Pose2D>("ICARUS_Rover_Pose",1000);

	ros::Rate loop_rate(100);
	std::clock_t    start;
	::icarus_rover_rc::ICARUS_Probe_Status Probe_Status;
	::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
	ICARUS_Diagnostic.header.frame_id = "ICARUS_Motion_Controller_Diagnostic";
	ICARUS_Diagnostic.System = ROVER;
	ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
	ICARUS_Diagnostic.Component = MOTION_CONTROLLER_NODE;
  
	temp1 = 0;
	while( ros::ok() && INITIALIZED)
	{
    
		start = std::clock();
	  	ros::spinOnce();
	  	loop_rate.sleep();
	  	if(armed_state == ARMED)
	  	{
			if(DEBUG_MODE >= INFORMATION) { printf("Armed State: ARMED\r\n"); }
		}
		else if(armed_state == DISARMED)
		{
			if(DEBUG_MODE >= INFORMATION) { printf("Armed State: DISARMED\r\n"); }
		}
		else
		{
			printf("Armed State: UNKNOWN!!!\r\n");
		}
	  	try
		{
			
			Steer_Command = STEER_SERVO_CENTER;
			Drive_Command = DRIVE_MOTOR_NEUTRAL;
			
			if (DEBUG_MODE == 1) { printf("$NAV,%d,%d,*\r\n",(int)Steer_Command,(int)Drive_Command);}
			//printf();
			if(1)
			{
				ICARUS_Diagnostic.Diagnostic_Type = COMMUNICATIONS;
				ICARUS_Diagnostic.Level = DEBUG;
				ICARUS_Diagnostic.Diagnostic_Message = NO_ERROR;
				//ICARUS_Diagnostic.Description =  "";
				if (DEBUG_MODE == 1) 
				{
					if(Gps_Valid == 0) { cout << "GPS: NO FIX." << endl;}
					else if(Gps_Valid == 1) { cout << "GPS: NAV READY." << endl; }
				}
				//Pub_ICARUS_Rover_Pose.publish(Rover_Pose);
			}
			
			
			else
			{
				ICARUS_Diagnostic.header.stamp = ros::Time::now();
				ICARUS_Diagnostic.Diagnostic_Type = COMMUNICATIONS;
				ICARUS_Diagnostic.Level = CAUTION;
				ICARUS_Diagnostic.Diagnostic_Message = DROPPING_PACKETS;
				Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
			}
			dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
     

      //Recharge_FSM(dtime);
			Probe_Status.header.stamp = ros::Time::now();
			Probe_Status.header.frame_id = "Probe_Status";
			Probe_Status.Probe_State = Probe_State;
			Probe_Status.Extended_Switch = Extended_Switch;
			Probe_Status.Retracted_Switch = Retracted_Switch;  
			Probe_Status.Probe_Error = Probe_Error;

    
			Pub_ICARUS_Probe_Status.publish(Probe_Status);  

			  //ICARUS Diagnostics Publisher
			  ICARUS_Diagnostic.header.stamp = ros::Time::now();
			  Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
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
			Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
		}
	}
	ICARUS_Diagnostic.header.stamp = ros::Time::now();
	ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
	ICARUS_Diagnostic.Level = SEVERE;
	ICARUS_Diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
	ICARUS_Diagnostic.Description = "Node Closed.";
	Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
  
}
