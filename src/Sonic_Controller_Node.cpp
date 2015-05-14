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
#include "icarus_rover_rc/ICARUS_Diagnostic.h"

using namespace std;

 
string SC_Device = "";
int Baud_Rate = -1;
double dtime = 0.0;
int sonic_node_rate = 0.0;
int ping_sensor_count = 0;
int DEBUG_MODE = 0;
int angle_min,angle_max,range_min,range_max,angle_increment;

int main(int argc, char **argv)
{
  int INITIALIZED = 0;
  ros::init(argc, argv, "Sonic_Controller");
  ros::NodeHandle nh("~");
  //nh.getParam("target_count",target_count);
  nh.getParam("sc_device",SC_Device);
  nh.getParam("baudrate",Baud_Rate);
  nh.getParam("rate",sonic_node_rate);
  nh.getParam("ping_sensor_count",ping_sensor_count);
  nh.getParam("DEBUG_MODE",DEBUG_MODE);
  nh.getParam("angle_min_deg",angle_min);
  nh.getParam("angle_max_deg",angle_max);
  nh.getParam("angle_increment_deg",angle_increment);
  nh.getParam("range_min_inch",range_min);
  nh.getParam("range_max_inch",range_max);
  
  int sc_device;
  struct termios oldtio,newtio;
  int buffer_length = 255;
	int message_complete = 0;
  int message_started = 0;
  char message[256];
  int message_index = 0;
  int res;
  int ping_distances[ping_sensor_count];
  int ping_index = 0;
  
  sc_device= open(SC_Device.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
  
  if (sc_device  == -1)
  {
    printf("ERROR: UNABLE TO OPEN SONIC CONTROLLER PORT ON %s.",SC_Device.c_str());
  }
  else
  {
    
    INITIALIZED = 1;
    tcgetattr(sc_device,&oldtio);
	  bzero(&newtio, sizeof(newtio));
	  cfsetospeed(&newtio, (speed_t)B115200);
    cfsetispeed(&newtio, (speed_t)B115200);
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;
    newtio.c_cflag &= CRTSCTS;
    newtio.c_cflag |= CREAD | CLOCAL;
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 5;
    cfmakeraw(&newtio);
	  tcflush(sc_device, TCIFLUSH);
	  tcsetattr(sc_device,TCSANOW,&newtio);
  }

  ros::Publisher Pub_ICARUS_Sonar_Scan = nh.advertise<sensor_msgs::LaserScan>("ICARUS_Sonar_Scan",1000);
  ros::Publisher Pub_ICARUS_Sonic_Controller_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Sonic_Controller_Diagnostic",1000);
  icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
  ros::Rate loop_rate(sonic_node_rate);
	std::clock_t    start;
  sensor_msgs::LaserScan Sonar_Scan;
  ICARUS_Diagnostic.header.frame_id = "ICARUS_Sonic_Controller_Diagnostic";
  ICARUS_Diagnostic.System = ROVER;
  ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
  ICARUS_Diagnostic.Component = SONIC_CONTROLLER_NODE;
	//LaserScan Sonar_Scan;
  start = std::clock();
	while( ros::ok() && INITIALIZED)
	{
    
	  
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  try
	  {
	    int wr;
      //wr = write(mc_device,"HELLO, WORLD!\r\n",15);
      char buf = '\0';
      char response[255];
      int index = 0;
      res = read(sc_device,&response,255);
      response[res] = 0;
      //if (res > 0) { printf("xxx.%s",response); }
      if (message_complete == 0)
      {
        if (message_complete == 1)
        {
          break;
        }
        string tempstr(response);
        for(int i = 0; i <tempstr.size();i++)
        {
          if (tempstr[i] == '$')
          {
            message_index = 0;
            memset(&message,0,256);
            sprintf(&message[message_index++],"$");
            message_complete = 0;
            message_started = 1;
          }
          else if ((tempstr[i] == '*') && (message_started == 1))
          {
            sprintf(&message[message_index++],"*");
            message_complete = 1;
            message_started = 0;
          }
          else if (message_started == 1)
          { 
            sprintf(&message[message_index++],"%c",tempstr[i]);
          }

        }
      }
      if (message_complete == 1) 
      {  
        message_complete = 0;
        //cout << message << endl; message_complete = 0;
        string tempstr(message);
        string delimiter = ",";
        int pos = 0;
        string token;
        
        string message_type = tempstr.substr(0,tempstr.find(delimiter));
        ping_index = 0;
        if( message_type.compare("$SON") ==0)
        {
          int skipme = 1;
          while(( pos = tempstr.find(delimiter)) != std::string::npos)
          {
            
            token = tempstr.substr(0,pos);
            
            if (skipme == 0)
            {
              ping_distances[ping_index++] = atol(token.c_str());
            
            }
            skipme = 0;
            tempstr.erase(0,pos+delimiter.length());
          }
           pos = tempstr.find("*");
          token = tempstr.substr(0,pos);
          
            ping_distances[ping_index++] = atol(token.c_str());
          
        }
        dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
        start = std::clock();
  
      }
      
      
      
      
      Sonar_Scan.header.stamp = ros::Time::now();
      Sonar_Scan.header.frame_id = "sonar_frame";
      Sonar_Scan.angle_min = (float)angle_min*3.145/180.0;
      Sonar_Scan.angle_max = (float)angle_max*3.145/180.0;
      Sonar_Scan.angle_increment = angle_increment*3.145/180.0;
      Sonar_Scan.range_min = range_min;
      Sonar_Scan.range_max = range_max;
      Sonar_Scan.scan_time = dtime;
      Sonar_Scan.ranges.resize(ping_sensor_count);
      for(int i = 0; i < ping_sensor_count;i++)
      {  
        if (ping_distances[i] < range_min) { ping_distances[i] = range_min; }
        else if (ping_distances[i] > range_max) { ping_distances[i] = range_max; }
        Sonar_Scan.ranges[i] = ping_distances[i]; 
      }
     
      Pub_ICARUS_Sonar_Scan.publish(Sonar_Scan);
      ICARUS_Diagnostic.header.stamp = ros::Time::now();
      ICARUS_Diagnostic.Diagnostic_Type = NO_ERROR;
      ICARUS_Diagnostic.Level = NO_ERROR;
      ICARUS_Diagnostic.Diagnostic_Message = NO_ERROR;
      Pub_ICARUS_Sonic_Controller_Diagnostic.publish(ICARUS_Diagnostic);
	      
	  }
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
      close(sc_device);
		ICARUS_Diagnostic.header.stamp = ros::Time::now();
		ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
		ICARUS_Diagnostic.Level = FATAL;
		ICARUS_Diagnostic.Diagnostic_Message = GENERAL_ERROR;
		Pub_ICARUS_Sonic_Controller_Diagnostic.publish(ICARUS_Diagnostic);	
	  }
  }
  close(sc_device);
  
}
