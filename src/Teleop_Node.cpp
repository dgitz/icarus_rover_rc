//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <dirent.h>
//#include <gps_common/conversions.h>
#include "icarus_rover_rc/Definitions.h"
#include "icarus_rover_rc/ICARUS_Diagnostic.h"
#define PI 3.14159265359


#include <sys/time.h>


using namespace std;

int DEBUG_MODE= 0;
double dtime = 0.0;
std::string last_Armed_Value = "";
std::string Armed_Value = "";
bool armed_status_changed = false;
::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;

void ICARUS_Rover_Diagnotic_Callback(const icarus_rover_rc::ICARUS_Diagnostic::ConstPtr& msg)
{
	try
	{
          if((msg->System == ROVER) && (msg->SubSystem == ROBOT_CONTROLLER) && (msg->Component == MOTION_CONTROLLER_NODE) &&
             (msg->Diagnostic_Type == REMOTE_CONTROL) && (msg->Level == INFORMATION))
          {
               if((msg->Diagnostic_Message == ROVER_ARMED) || (msg->Diagnostic_Message == ROVER_DISARMED))
               {
           
                    Armed_Value = msg->Description;
               }
          }
          
          
	}
	catch(exception& e)
	{
	}
	
}
int main(int argc, char **argv)
{
	
	
	
	int INITIALIZED = 1;

	ros::init(argc, argv, "Teleop_Node");
	ros::NodeHandle nh("~");
	
	ICARUS_Diagnostic.System = REMOTE_CONTROL;
	ICARUS_Diagnostic.SubSystem = ENTIRE_SYSTEM;
	ICARUS_Diagnostic.Component = ENTIRE_SUBSYSTEM;
	ICARUS_Diagnostic.Diagnostic_Type = NO_ERROR;
	ICARUS_Diagnostic.Level = INFORMATION;
	ICARUS_Diagnostic.Diagnostic_Message = INITIALIZING;
	
	ros::Publisher Pub_ICARUS_Teleop_Node_Diagnostic;
     ros::Subscriber Sub_ICARUS_Teleop_Node_Diagnostic;
     Pub_ICARUS_Teleop_Node_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Teleop_Node_Diagnostic",1000);
     Sub_ICARUS_Teleop_Node_Diagnostic = nh.subscribe<icarus_rover_rc::ICARUS_Diagnostic>("/Motion_Controller_Node/ICARUS_Motion_Controller_Diagnostic",1000,ICARUS_Rover_Diagnotic_Callback);	

	
	
	ros::Rate loop_rate(50);
	while( ros::ok() && INITIALIZED)
	{
		
	  	ros::spinOnce();
		loop_rate.sleep();
          //printf("l: %s c: %s =: %d\r\n",last_Armed_Value.c_str(),Armed_Value.c_str(),last_Armed_Value.compare(Armed_Value));
          if(last_Armed_Value.compare(Armed_Value) != 0)
          {
               armed_status_changed = true;
          }
          else
          {
               armed_status_changed = false;
          }
          if(armed_status_changed == true)
          {
               //armed_status_changed = false;
               printf("%s\r\n",Armed_Value.c_str());
          }		
         last_Armed_Value = Armed_Value;
 
         Pub_ICARUS_Teleop_Node_Diagnostic.publish(ICARUS_Diagnostic);
		
	}
}
