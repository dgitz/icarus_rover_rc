//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
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


#define LINE1 254
#define BACKLIGHT_ON 157
#define BACKLIGHT_OFF 128
using namespace std;

//Communication Variables
int token_index = 0;
string DIAG_device = "";
int Baud_Rate = -1;


double dtime = 0.0;
int main(int argc, char **argv)
{
  int INITIALIZED = 0;
  ros::init(argc, argv, "Diagnostic Node");
  ros::NodeHandle nh;
  //nh.getParam("target_count",target_count);
  nh.getParam("diag_device",DIAG_device);
  nh.getParam("baudrate",Baud_Rate);
  int diag_device;
  struct termios oldtio,newtio;
	int res;
  
  diag_device= open(DIAG_device.c_str(),O_RDWR | O_NOCTTY );

  if (diag_device  == -1)
  {
    ROS_INFO("ERROR: UNABLE TO OPEN SERIAL PORT.");
  }
  else
  { 
    INITIALIZED = 1;  
    tcgetattr(diag_device,&oldtio);
	  bzero(&newtio, sizeof(newtio));
	  cfsetospeed(&newtio, (speed_t)B9600);
    cfsetispeed(&newtio, (speed_t)B9600);
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;
    newtio.c_cflag &= CRTSCTS;
    newtio.c_cflag |= CREAD | CLOCAL;
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 5;
    cfmakeraw(&newtio);
	  tcflush(diag_device, TCIFLUSH);
	  tcsetattr(diag_device,TCSANOW,&newtio);
  }  

  //ros::Subscriber Sub_ICARUS_Probe_Command_Callback = nh.subscribe("ICARUS_Probe_Command", 1000, ICARUS_Probe_Command_Callback);
  //ros::Subscriber Pub_Rover_Control = nh.subscribe<sensor_msgs::Joy>("ICARUS_Rover_Control",1000,ICARUS_Rover_Control_Callback);
  ros::Rate loop_rate(100);
	std::clock_t    start;
  while( ros::ok() && INITIALIZED)
	{
    
	  start = std::clock();
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  try
	  {
      char cmd[255];
      int wr;
      /*memset(cmd,'\0',sizeof cmd);
      sprintf(cmd,"%d",LINE1);
      wr = write(diag_device,cmd,sizeof(cmd)-1);
	          
      memset(cmd,'\0',sizeof cmd);
      sprintf(cmd,"%d",BACKLIGHT_ON);
      wr = write(diag_device,cmd,sizeof(cmd)-1);*/
	    
      memset(cmd,'\0',sizeof cmd);
      sprintf(cmd,"HELLO! WORLD",BACKLIGHT_ON);
      wr = write(diag_device,cmd,sizeof(cmd)-1);
	    
      //printf();
      dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
      }
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
      close(diag_device);
	  }
  }
  close(diag_device);
  
}
