//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Odometry.h>
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
//#include "icarus_rover_rc/ICARUS_Diagnostic.h"
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
using namespace std;


//Communication Variables

//Operation Variables
string Mode = "";

//Program Variables
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mapping_Node");
 	ros::NodeHandle nh("~");
  	nh.getParam("Mode",Mode);
	ros::Rate loop_rate(100);
	std::clock_t    start;	

	tf::TransformListener tf(ros::Duration(10));
	costmap_2d::Costmap2DROS costmap("my_costmap",tf);
	while(ros::ok())
	{
		start = std::clock();
		ros::spinOnce();
		loop_rate.sleep();
		try
		{
		}
		catch(const std::exception& ex)
		{
			ROS_INFO("ERROR:%s",ex.what());
		}
	}
  
}
