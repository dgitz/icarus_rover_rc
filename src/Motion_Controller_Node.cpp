//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <dirent.h>
#include <iomanip>
#include <ctime>
#include "icarus_rover_rc/Definitions.h"
#include "icarus_rover_rc/ICARUS_Probe_Status.h"
#include "icarus_rover_rc/ICARUS_Probe_Command.h"
#include "icarus_rover_rc/RoboPiLib.h"

using namespace std;

int Extended_Switch = 0;
int Retracted_Switch = 0;
int Forward_Timeout = 0;
int Reverse_Timeout = 0;
int Recharge_Command = RECHARGE_WAIT;
double forward_timer = 0.0;
double reverse_timer = 0.0;

 
string MC_Device = "";
int Baud_Rate = -1;
int Probe_State = PROBE_RETRACTED;
int Probe_Error = NO_ERROR;
double dtime = 0.0;

void ICARUS_Probe_Command_Callback(const icarus_rover_rc::ICARUS_Probe_Command::ConstPtr& msg)  //Process ICARUS Probe Command Message
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  Recharge_Command = 0;//msg.Charge_Command;
  
}
int Read_Distance(int Sonar_Pin)
{
  int dist = 0;
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
      servoWrite(WINCH_MOTOR_PIN, WINCH_MOTOR_FORWARD);
      break;
    case PROBE_MOVING_REVERSE:
      reverse_timer += dtime;
      if (Retracted_Switch == true) 
      { 
        Probe_State = PROBE_RETRACTED; 
      }
      servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_REVERSE);
      break;
    case PROBE_EXTENDED:
      forward_timer = 0.0;
      reverse_timer = 0.0;
      if (Recharge_Command == RECHARGE_STOP)
      {
        Probe_State = PROBE_MOVING_REVERSE;
      }
      servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
    case PROBE_RETRACTED:
      forward_timer = 0.0;
      reverse_timer = 0.0;
      if (Recharge_Command == RECHARGE_START) 
      { 
        Probe_State = PROBE_MOVING_FORWARD; 
        Probe_Error = NO_ERROR;
      }
      servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
    default:
      servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
  }
  if (forward_timer > FORWARD_TIME_LIMIT){ Probe_Error = EXTENSION_ERROR; }
  if (reverse_timer > REVERSE_TIME_LIMIT){ Probe_State = RETRACTION_ERROR; }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Motion_Controller");
  ros::NodeHandle nh;
  //nh.getParam("target_count",target_count);
  nh.getParam("mc_device",MC_Device);
  nh.getParam("baudrate",Baud_Rate);
  ros::Subscriber Sub_ICARUS_Probe_Command_Callback = nh.subscribe("ICARUS_Probe_Command", 1000, ICARUS_Probe_Command_Callback);
  ros::Publisher Pub_ICARUS_Probe_Status = nh.advertise<icarus_rover_rc::ICARUS_Probe_Status>("ICARUS_Probe_Status", 1000);  
  ros::Publisher Pub_ICARUS_Sonar_Scan = nh.advertise<sensor_msgs::LaserScan>("ICARUS_Sonar_Scan",1000);
  ros::Rate loop_rate(1000);
	std::clock_t    start;
  ::icarus_rover_rc::ICARUS_Probe_Status Probe_Status;
  sensor_msgs::LaserScan Sonar_Scan;
	//LaserScan Sonar_Scan;
  pinMode(EXTENDED_SWITCH_PIN,  INPUT);
  pinMode(RETRACTED_SWITCH_PIN, INPUT);
  pinMode(WINCH_MOTOR_PIN,SERVO);
  ::RoboPiInit(const_cast<char*>(MC_Device.c_str()), Baud_Rate); // device = "/dev/ttyAMA0", bps = 9600..115200 matching RoboPi config

	while( ros::ok() && true)
	{
    
	  start = std::clock();
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  try
	  {
	    
	    dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
      if (digitalRead(EXTENDED_SWITCH_PIN)==0)
      {
        Extended_Switch = true;
      }
      else
      {
        Extended_Switch = false;
      }
      if (digitalRead(RETRACTED_SWITCH_PIN)==0)
      {
        Retracted_Switch = true;
      }
      else
      {
        Retracted_Switch = false;
      }

      Recharge_FSM(dtime);
      Probe_Status.header.stamp = ros::Time::now();
      Probe_Status.header.frame_id = "Probe_Status";
      Probe_Status.Probe_State = Probe_State;
      Probe_Status.Extended_Switch = Extended_Switch;
      Probe_Status.Retracted_Switch = Retracted_Switch;  
      Probe_Status.Probe_Error = Probe_Error;

      Sonar_Scan.header.stamp = ros::Time::now();
            
      
      Pub_ICARUS_Sonar_Scan.publish(Sonar_Scan);
	    Pub_ICARUS_Probe_Status.publish(Probe_Status);    
	  }
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
	  }
  }
  RoboPiExit(); // close the serial device
}
