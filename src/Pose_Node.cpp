//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
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
#include "icarus_rover_rc/ICARUS_Diagnostic.h"
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
using namespace std;


#define MOUSEID "/dev/input/event0"
#define MOUSEID1 "/dev/input/event1"
//#define MOUSEID2 "/dev/input/event2"
#define MOUSEID2 "/dev/input/event1"
//Communication Variables

//Operation Variables
string Mode = "";

//Program Variables
double dtime = 0.0;
double Pose_X;
double Pose_Y;
double Pose_Theta;
void ICARUS_Rover_Pose_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{

	static tf::TransformBroadcaster odom_br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->x,msg->y,0.0));
	tf::Quaternion q;
	q.setRPY(0.0,0.0,msg->theta);
	transform.setRotation(q);
	odom_br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","odom"));
	Pose_X = msg->x;
	Pose_Y = msg->y;
	Pose_Theta = msg->theta;
	
	static tf::TransformBroadcaster scan_br;
	tf::Transform scan_transform;
	scan_transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
	tf::Quaternion q2;
	q2.setRPY(0,0,0);
	scan_transform.setRotation(q2);
	scan_br.sendTransform(tf::StampedTransform(scan_transform,ros::Time::now(),"base_link","scan"));
	/*
	tf::TransformBroadcaster scan_broadcaster;
	geometry_msgs::Quaternion scan_quat = tf::createQuaternionMsgFromYaw(0.0);
	geometry_msgs::TransformStamped scan_trans;
	scan_trans.header.stamp = ros::Time::now();
	scan_trans.header.frame_id = "/scan";
	scan_trans.child_frame_id = "/base_link";
	scan_trans.transform.translation.x = 0.0;
	scan_trans.transform.translation.y = 0.0;
	scan_trans.transform.translation.z = -.0;
	scan_trans.transform.rotation = scan_quat;
	scan_broadcaster.sendTransform(scan_trans);
	*/
	tf::TransformBroadcaster scan_broadcaster;
	//scan_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.0,0.0,0.2)),
	//	ros::Time::now(),"/base_link","/scan"));

	printf("Got a Pose x: %f y: %f theta: %f\r\n",msg->x,msg->y,msg->theta);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mapping_Node");
  ros::NodeHandle nh("~");
  
  nh.getParam("Mode",Mode);
  
    
  ros::Publisher Pub_ICARUS_Pose_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Pose_Diagnostic",1000);
  ros::Rate loop_rate(100);
  std::clock_t    start;
  ros::Publisher Pub_ICARUS_Rover_Odom;    


   ::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
  ICARUS_Diagnostic.header.frame_id = "ICARUS_Pose_Diagnostic";
  ICARUS_Diagnostic.System = ROVER;
  ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
  ICARUS_Diagnostic.Component = POSE_NODE;
  
	int fd;
	struct input_event ie;
	ssize_t n;
	if((fd = open(MOUSEID2,O_RDONLY)) == -1)
	{
		cout << "Couldn't Open Mouse: " << fd << endl;
	}
	int x = 0;
	int y = 0;
	int dx = 0;
	int dy = 0;
	while(ros::ok())
	{
    
		start = std::clock();
		ros::spinOnce();
		loop_rate.sleep();
		
		
		n = read(fd,&ie,sizeof ie);
		if(n>0)
		/*{
			unsigned char *ptr = (unsigned char*)&ie;
			//x = (char)ptr[4]; y = (char)ptr[5];
			//printf("x: %d y: %d\r\n",x,y);
			for(int i = 0; i < sizeof(ie);i++)
			{
				printf("%02X ",*ptr++);
			}
			printf("\n");*/

		{
			//cout << "Type: " << ie.type << " Code: " << ie.code << " Value: " << ie.value << endl; 
			if(ie.type == 0x02)
			{
				if(ie.code == 0x00)
				{	
					x+=ie.value; dx = ie.value;
				}
				else
				{
					dx = 0;
				}
				if(ie.code == 0x01)
				{   
					y+=ie.value; 
					dy = ie.value;
				}
				else
				{
					dy = 0;
				}
				
				
			}
			else
			{
				dx = 0;
				dy = 0;
			}
			
			
		}
		cout << "dX: " << dx << " dY: " << dy << endl;
		
	  try
	  {
	    
	    dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
            ICARUS_Diagnostic.header.stamp = ros::Time::now();
            Pub_ICARUS_Pose_Diagnostic.publish(ICARUS_Diagnostic);
	    nav_msgs::Odometry Rover_Odometry;
	    Rover_Odometry.header.stamp = ros::Time::now();
	    Rover_Odometry.header.frame_id = "/odom";
	    Rover_Odometry.pose.pose.position.x = Pose_X;
	    Rover_Odometry.pose.pose.position.y = Pose_Y;
	    Rover_Odometry.pose.pose.position.z = 0.0;
	    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Pose_Theta);
	    Rover_Odometry.pose.pose.orientation = odom_quat;
	    Pub_ICARUS_Rover_Odom.publish(Rover_Odometry);
	  }
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
            
            //ICARUS Diagnostics Publisher
            ICARUS_Diagnostic.header.stamp = ros::Time::now();
            ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
            ICARUS_Diagnostic.Level = FATAL;
            ICARUS_Diagnostic.Diagnostic_Message = GENERAL_ERROR;
            ICARUS_Diagnostic.Description = ex.what();
            Pub_ICARUS_Mapping_Diagnostic.publish(ICARUS_Diagnostic);
	  }
  }

  
}
