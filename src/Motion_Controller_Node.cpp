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
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include "icarus_rover_rc/Definitions.h"
#include "icarus_rover_rc/ICARUS_Probe_Status.h"
#include "icarus_rover_rc/ICARUS_Probe_Command.h"

int servoWrite(int pin,int value);
int digitalRead(int pin);
int pingRead(int pin);
void test_uart();

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
  int INITIALIZED = 0;
  ros::init(argc, argv, "Motion_Controller");
  ros::NodeHandle nh;
  //nh.getParam("target_count",target_count);
  nh.getParam("mc_device",MC_Device);
  nh.getParam("baudrate",Baud_Rate);
  int mc_device;
  struct termios oldtio,newtio;
	char buf[255];
  int res;
  
  mc_device= open(MC_Device.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (mc_device  == -1)
  {
    ROS_INFO("ERROR: UNABLE TO OPEN SERIAL PORT.");
  }
  else
  {
    
    INITIALIZED = 1;
    tcgetattr(mc_device,&oldtio);
	  bzero(&newtio, sizeof(newtio));
	  newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    //newtio.c_cflag &= ~(IXON | IXOFF | IXANY);
	  newtio.c_iflag = IGNPAR;
	  newtio.c_oflag = 0;
	  newtio.c_lflag = 0;
	  newtio.c_cc[VTIME]    = 0;
	  newtio.c_cc[VMIN]     = 1;

	  tcflush(mc_device, TCIFLUSH);
	  tcsetattr(mc_device,TCSANOW,&newtio);
  }

  ros::Subscriber Sub_ICARUS_Probe_Command_Callback = nh.subscribe("ICARUS_Probe_Command", 1000, ICARUS_Probe_Command_Callback);
  ros::Publisher Pub_ICARUS_Probe_Status = nh.advertise<icarus_rover_rc::ICARUS_Probe_Status>("ICARUS_Probe_Status", 1000);  
  ros::Publisher Pub_ICARUS_Sonar_Scan = nh.advertise<sensor_msgs::LaserScan>("ICARUS_Sonar_Scan",1000);
  ros::Rate loop_rate(1000);
	std::clock_t    start;
  ::icarus_rover_rc::ICARUS_Probe_Status Probe_Status;
  sensor_msgs::LaserScan Sonar_Scan;
	//LaserScan Sonar_Scan;
  
	while( ros::ok() && INITIALIZED)
	{
    
	  start = std::clock();
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  try
	  {
	    int wr;
      //wr = write(mc_device,"HELLO, WORLD!\r\n",15);
      res = read(mc_device,buf,255);
      if(res > 0) 
		  {
        printf("%s\r\n",buf);
      }
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

      //Recharge_FSM(dtime);
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
      close(mc_device);
	  }
  }
  close(mc_device);
  
}
int servoWrite(int pin,int value)
{
  return 0;
}
int digitalRead(int pin)
{
  return -1;
}
int pingRead(int pin)
{
  return 0;
}
