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


using namespace std;

 
string SC_Device = "";
int Baud_Rate = -1;
double dtime = 0.0;
int sonic_node_rate = 0.0;

int main(int argc, char **argv)
{
  int INITIALIZED = 0;
  ros::init(argc, argv, "Sonic_Controller");
  ros::NodeHandle nh;
  //nh.getParam("target_count",target_count);
  nh.getParam("sc_device",SC_Device);
  nh.getParam("baudrate",Baud_Rate);
  nh.getParam("rate",sonic_node_rate);
  int sc_device;
  struct termios oldtio,newtio;
  int buffer_length = 255;
	char buf[255];
  int res;
  
  sc_device= open(SC_Device.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
  
  if (sc_device  == -1)
  {
    ROS_INFO("ERROR: UNABLE TO OPEN SONIC CONTROLLER PORT.");
  }
  else
  {
    
    INITIALIZED = 1;
    tcgetattr(sc_device,&oldtio);
	  bzero(&newtio, sizeof(newtio));
	  newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio.c_cflag &= ~(IXON | IXOFF | IXANY);
	  newtio.c_iflag = IGNPAR;
	  newtio.c_oflag = 0;
	  newtio.c_lflag = 0;
	  newtio.c_cc[VTIME]    = 0;
	  newtio.c_cc[VMIN]     = 1;

	  tcflush(sc_device, TCIFLUSH);
	  tcsetattr(sc_device,TCSANOW,&newtio);
  }

  ros::Publisher Pub_ICARUS_Sonar_Scan = nh.advertise<sensor_msgs::LaserScan>("ICARUS_Sonar_Scan",1000);
  ros::Rate loop_rate(sonic_node_rate);
	std::clock_t    start;
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
      res = read(sc_device,buf,buffer_length);
      if(res > 0) 
		  {
        string tempstr(buf);
        int start_char_count = 0;
        int end_char_count = 0;
    
        for(int i = 0; i < tempstr.size();i++)
        {
          if (tempstr[i] == '$') {start_char_count++;}
          if (tempstr[i] == '*') {end_char_count++;}
        }
        //int start_char = count(tempstr.begin(),tempstr.end(),'$');
        //int end_char = count(tempstr.begin(),tempstr.end(),'*');
        
        printf("%d...%d...%d...%s\r\n",tempstr.size(),start_char_count,end_char_count,tempstr.c_str());
        
        string msg;
        //for(int i =0; i < tempstr.length(); i++)
        //{
         // if (tempstr.find("$")) { printf("Here."); }
        //}
      }
      
	    dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
      Sonar_Scan.header.stamp = ros::Time::now();
            
      
      Pub_ICARUS_Sonar_Scan.publish(Sonar_Scan);
	      
	  }
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
      close(sc_device);
	  }
  }
  close(sc_device);
  
}
