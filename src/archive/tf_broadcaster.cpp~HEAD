#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
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

std::string turtle_name;
void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->x,msg->y,0.0));
	tf::Quaternion q;
	q.setRPY(0,0,msg->theta);
	transform.setRotation(q);
	printf("Got a Pose\r\n");
	br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world",turtle_name));
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"tf_broadcaster");
	ros::NodeHandle node("~");
	turtle_name = "turtle1";
	ros::Rate loop_rate(100);
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
	tf::Quaternion q;
	q.setRPY(0,0,0.0);
	transform.setRotation(q);
	//ros::Subscriber sub =  node.subscribe<geometry_msgs::Pose2D>("/Matlab_Node/ICARUS_SimRover_Pose",1000,poseCallback);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		printf("Running.\r\n");
		br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","laser"));
		br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","map"));
	}
}
