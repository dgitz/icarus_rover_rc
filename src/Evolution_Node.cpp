//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
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
#include "icarus_rover_rc/Definitions.h"
#include <icarus_rover_rc/Evolution.h>
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace std;

int main(int argc, char **argv)
{ 


  

  ros::init(argc, argv, "Evolution Node");
  ros::NodeHandle nh;
  std::clock_t lastclock;
  double dtime = 0.0;
  ros::Rate loop_rate(1000);
	
  //nh.getParam("target_count",target_count);
  //ros::Publisher Pub_ICARUS_Target_Status = nh.advertise<icarus_rover_rc::ICARUS_Target_Status>("ICARUS_Target_Status", 1000);  
  while( ros::ok() && true)
	{
    
	  lastclock = std::clock();
	  ros::spinOnce();
	  loop_rate.sleep();
    Gene mygene(3);
    ROS_INFO("%d",mygene.getGene());
	  try
    {
      dtime = (std::clock() - lastclock) / (double)(CLOCKS_PER_SEC /1);
    
    } 
	     
	    
	  
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
	  }
  }  
}


