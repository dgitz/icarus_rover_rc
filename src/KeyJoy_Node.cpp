/* Author: David Gitz
   Date: 7-March-2016
   Purpose: Converts ros keyboard topic to joystick topic.
   Usage: See Launch_UserControl.launch

*/
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
#include "keyboard/Key.h"
#include <sensor_msgs/Joy.h>
#define KEY_A 97 
#define KEY_D 100
#define KEY_UP 273
#define KEY_DOWN 274
#define KEY_LEFT 276
#define KEY_RIGHT 275
#define KEY_SPACE 32

#define JOY_STEER_AXIS  0
#define JOY_THROTTLE_AXIS 3
#define JOY_DISARM_BUTTON  14
#define JOY_ARM_BUTTON 12

float Steer_Value = 0.0;
float Throttle_Value = 0.0;
int Armed_State = 0;
int armed_state_changed = 0;
int axis_changed = 0;



#include <sys/time.h>
sensor_msgs::Joy joy_command;
ros::Publisher Pub_Joy;          
void KeyDown_Callback(const keyboard::Key::ConstPtr& msg)
{
     if(msg->code == KEY_A)
     {
          Armed_State = 1;
          armed_state_changed = 1;
          
     }
     else if((msg->code == KEY_D) || (msg->code == KEY_SPACE))
     {
          Armed_State = 0;
          Steer_Value = 0.0;
          Throttle_Value = 0.0;
          armed_state_changed = 1;
          axis_changed = 1;
     }
     else if(msg->code == KEY_UP)
     {
          float temp = Throttle_Value;
          temp += 0.05;
          if(temp < 1.0) { Throttle_Value = temp; } 
          else { Throttle_Value = 1.0; }
          axis_changed = 1;         
     }
     else if(msg->code == KEY_DOWN)
     {
          float temp = Throttle_Value;
          temp -= 0.05;
          if(temp > -1.0) { Throttle_Value = temp; } 
          else { Throttle_Value = -1.0; }
          axis_changed = 1;
     }
     else if(msg->code == KEY_LEFT)
     {
          float temp = Steer_Value;
          temp -= 0.05;
          if(temp > -1.0) { Steer_Value = temp; } 
          else { Steer_Value = -1.0; }
          axis_changed = 1;
     }
     else if(msg->code == KEY_RIGHT)
     {
          float temp = Steer_Value;
          temp += 0.05;
          if(temp < 1.0) { Steer_Value = temp; }
          else{ Steer_Value = 1.0; }
          axis_changed = 1; 
     }
     if(armed_state_changed == 1)
     {
          if(Armed_State == 0)
          {
               joy_command.buttons[JOY_DISARM_BUTTON] = 1;
               joy_command.buttons[JOY_ARM_BUTTON] = 0;
          }
          else
          {
               joy_command.buttons[JOY_DISARM_BUTTON] = 0;
               joy_command.buttons[JOY_ARM_BUTTON] = 1;
          }

          armed_state_changed = 0;     
     }
     if(axis_changed == 1)
     {
          joy_command.axes[JOY_STEER_AXIS] = Steer_Value*-1;;
          joy_command.axes[JOY_THROTTLE_AXIS] = Throttle_Value;
          axis_changed = 0;
     }

     Pub_Joy.publish(joy_command);

}
int main(int argc, char **argv)
{
	
	
	
	ros::init(argc, argv, "KeyJoy_Node");
	ros::NodeHandle nh("/");
     joy_command.buttons.resize(20);
     joy_command.axes.resize(8);
     ros::Subscriber Sub_KeyboardDown = nh.subscribe<keyboard::Key>("/keyboard/keydown",1000,KeyDown_Callback);
	Pub_Joy = nh.advertise<sensor_msgs::Joy>("joy",1000);
	
     ros::Rate loop_rate(50);
	while( ros::ok())
	{
          

	  	ros::spinOnce();
	  	loop_rate.sleep();
          
		
	}
	
}
