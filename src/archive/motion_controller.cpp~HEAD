//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <dirent.h>
#include <iomanip>
#include <ctime>

#define FORWARD_SWITCH_PIN -1
#define REVERSE_SWITCH_PIN -1
#define WINCH_MOTOR_PIN -1
#define FORWARD_TIME_LIMIT -1
#define REVERSE_TIME_LIMIT -1
#define WINCH_MOTOR_FORWARD -1
#define WINCH_MOTOR_REVERSE -1
#define WINCH_MOTOR_NEUTRAL -1

//#define PROBE_STOPPED = 0
#define PROBE_MOVING_FORWARD = 1
#define PROBE_MOVING_REVERSE = 2
#define PROBE_EXTENDED = 3
#define PROBE_RETRACTED = 4
#define PROBE_ERROR = 5


using namespace std;

int Forward_Switch = false;
int Reverse_Switch = false;
int Forward_Timeout = false;
int Reverse_Timeout = false;
int Recharge_Start = false;
int Recharge_Stop = false;
double forward_timer = 0.0;
double reverse_timer = 0.0;

 
string MC_Device = "";
int Baud_Rate = -1;
int Probe_State = PROBE_RETRACTED;

void Command_Callback(const std_msgs::String::ConstPtr& msg)  //Process all ICARUS commands
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
void Recharge_FSM(double dtime)
{
  switch Probe_State
  {
    case PROBE_MOVING_FORWARD:
      forward_timer += dtime;
      if (forward_switch == true)
      { 
        Probe_State = PROBE_EXTENDED; 
      }
      servoWrite(WINCH_MOTOR_PIN, WINCH_MOTOR_FORWARD);
      break;
    case PROBE_MOVING_REVERSE:
      reverse_timer += dtime;
      if (reverse_switch == true) 
      { 
        Probe_State = PROBE_REVERSED; 
      }
      servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_REVERSE);
      break;
    case PROBE_EXTENDED:
      forward_timer = 0.0;
      reverse_timer = 0.0;
      if (Recharge_Stop == true)
      {
        Probe_State = PROBE_MOVING_REVERSE;
        Recharge_Stop = false;
      }
      servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
    case PROBE_RETRACTED:
      forward_timer = 0.0;
      reverse_timer = 0.0;
      if (Recharge_Start == true) 
      { 
        Probe_State = PROBE_MOVING_FORWARD; 
        Recharge_Start = false;
      }
      servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
    case PROBE_ERROR:
      servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
    default:
      servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
  }
  if (forward_timer > FORWARD_TIME_LIMIT){ Probe_State = PROBE_ERROR; }
  if (reverse_timer > REVERSE_TIME_LIMIT){ Probe_State = PROBE_ERROR; }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_controller");
  //nh.getParam("target_count",target_count);
  nh.getParam("mc_device",MC_Device);
  nh.getParam("baudrate",Baud_Rate);

  ros::Rate loop_rate(1000);
	std::clock_t    start;
	
  pinMode(FORWARD_SWITCH_PIN,  INPUT);
  pinMode(REVERSE_SWITCH_PIN, INPUT);
  pinMode(WINCH_MOTOR_PIN,SERVO);
  RoboPiInit(mc_device, baudrate); // device = "/dev/ttyAMA0", bps = 9600..115200 matching RoboPi config

	while( ros::ok() && true)
	{
    
	  start = std::clock();
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  try
	  {
	    
	    dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
      if (digitalRead(FORWARD_SWITCH_PIN)==0)
      {
        Forward_Switch = true;
      }
      else
      {
        Forward_Switch = false;
      }
      if (digitalRead(REVERSE_SWITCH_PIN)==0)
      {
        Reverse_Switch = true;
      }
      else
      {
        Reverse_Switch = false;
      }


      Recharge_FSM(dtime); 
	    
	  }
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
	  }
  }
  RoboPiExit(); // close the serial device
}
