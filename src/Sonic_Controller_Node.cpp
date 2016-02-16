//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
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

#define PI 3.14159265359
using namespace std;

 
double dtime = 0.0;
int sonic_node_rate = 0.0;
int ping_sensor_count = 0;
int DEBUG_MODE = 0;
int angle_min,angle_max,angle_increment;
double range_min = 0.0;
double range_max = 0.0;
int points_per_beam = 1;
double beam_width = 30.0;
string Operation_Mode = "";
//Communication Variables
int token_index = 0;
string SC_Device = "";
int Baud_Rate = -1;
int in_message_started = 0;
int in_message_completed = 0;
int in_message_ready = 0;
//Sonar Variables
int SonarDistance_1 = 0;
int SonarDistance_2 = 0;
int SonarDistance_3 = 0;
int SonarDistance_4 = 0;
int SonarDistance_5 = 0;
int SonarDistance_6 = 0;
int SonarDistance_7 = 0;
int SonarDistance_8 = 0;
int SonarDistance_9 = 0;
int SonarDistance_10 = 0;
int SonarDistance_11 = 0;
int SonarDistance_12 = 0;
int SonarDistance_13 = 0;

int hex2dec(char);
int hex2dec(char,char);
ros::Publisher Pub_ICARUS_Sonar_Scan;
uint32_t sequence_counter = 0;

void ICARUS_SimSonar_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	printf("Got a scan\r\n");
	sensor_msgs::LaserScan Scan;
	Scan.header.stamp = ros::Time::now();
	Scan.header.frame_id = "/laser";
	Scan.header.seq = sequence_counter;
	sequence_counter++;
	Scan.angle_min = msg->angle_min;
	Scan.angle_max = msg->angle_max;
	Scan.range_min = msg->range_min;
	Scan.range_max = 50.0;
	Scan.scan_time = msg->scan_time;
	Scan.angle_increment = (Scan.angle_max-Scan.angle_min)/130.0;
	printf("Start: %f Stop: %f Inc: %f\r\n",msg->angle_min,msg->angle_max,Scan.angle_increment);
	Scan.ranges.resize(130.0);
	Scan.ranges = msg->ranges;

	Pub_ICARUS_Sonar_Scan.publish(Scan);
	
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"Sonic_Controller");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(100);
	ros::Publisher Pub_ICARUS_Sonic_Controller_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Sonic_Controller_Diagnostic",1000);
  	icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
	ICARUS_Diagnostic.System = ROVER;
  	ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
  	ICARUS_Diagnostic.Component = SONIC_CONTROLLER_NODE;
	ICARUS_Diagnostic.Diagnostic_Type = NO_ERROR;
	ICARUS_Diagnostic.Level = INFORMATION;
	ICARUS_Diagnostic.Diagnostic_Message = INITIALIZING;
	Pub_ICARUS_Sonic_Controller_Diagnostic.publish(ICARUS_Diagnostic);
	int INITIALIZED = 0;

  	//nh.getParam("target_count",target_count);
  	
  	nh.getParam("DEBUG_MODE",DEBUG_MODE);
  	printf("Got Debug Mode: %d\r\n",DEBUG_MODE);
	nh.getParam("Operation_Mode",Operation_Mode); //Should be: SIM, LIVE

	
	int sc_device;
  	struct termios oldtio,newtio;
  	int buffer_length = 255;
	int message_complete = 0;
 	 int message_started = 0;
  	char message[256];
  	int message_index = 0;
  	int res;
  	float ping_distances[ping_sensor_count];
  	int ping_index = 0;
	if(Operation_Mode=="LIVE")
	{
		nh.getParam("sc_device",SC_Device);
		printf("Got sc_device: %s\r\n",SC_Device.c_str());
		nh.getParam("baudrate",Baud_Rate);
		printf("Got baudrate: %d\r\n",Baud_Rate);
		sc_device= open(SC_Device.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
		nh.getParam("points_per_beam",points_per_beam);
		nh.getParam("beam_width",beam_width);
		if (sc_device  == -1)
		{
				printf("ERROR: UNABLE TO OPEN SONIC CONTROLLER PORT ON %s.",SC_Device.c_str());
		}
		else
		{
			printf("Serial Socket: %d\r\n",sc_device);
			
			INITIALIZED = 1;
			tcgetattr(sc_device,&oldtio);
			bzero(&newtio, sizeof(newtio));
			newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
			//newtio.c_cflag &= ~(IXON | IXOFF | IXANY);
			newtio.c_iflag = IGNPAR;
			newtio.c_oflag = 0;
			newtio.c_lflag = 0;
			newtio.c_cc[VTIME]    = 1;
			newtio.c_cc[VMIN]     = 5;
			tcflush(sc_device, TCIFLUSH);
			tcsetattr(sc_device,TCSANOW,&newtio);
		}
	}
	
	ros::Subscriber Sub_ICARUS_Sonar_Scan;
	if(Operation_Mode == "LIVE")
	{
	
		nh.getParam("rate",sonic_node_rate);
		printf("Got Rate: %d\r\n",sonic_node_rate);
		nh.getParam("ping_sensor_count",ping_sensor_count);
		printf("Got Ping Sonar Count: %d\r\n",ping_sensor_count);
		nh.getParam("angle_min_deg",angle_min);
		nh.getParam("angle_max_deg",angle_max);
		nh.getParam("angle_increment_deg",angle_increment);
		nh.getParam("range_min_m",range_min);
		nh.getParam("range_max_m",range_max);
		//printf("Min Range: %f Max Range: %f",range_min,range_max);
		Pub_ICARUS_Sonar_Scan = nh.advertise<sensor_msgs::LaserScan>("ICARUS_Sonar_Scan",1000);
	}
	else if(Operation_Mode == "SIM")
	{
		INITIALIZED = 1;
		Sub_ICARUS_Sonar_Scan = nh.subscribe<sensor_msgs::LaserScan>("/Matlab_Node/ICARUS_SimSonar_Scan",1000,ICARUS_SimSonar_Scan_Callback);
		Pub_ICARUS_Sonar_Scan = nh.advertise<sensor_msgs::LaserScan>("ICARUS_Sonar_Scan",1000);
	}
	tf::TransformBroadcaster scan_broadcaster;
  	
  	//ros::Rate loop_rate(sonic_node_rate);
	std::clock_t    start;
  	sensor_msgs::LaserScan Sonar_Scan;

	//LaserScan Sonar_Scan;
  	start = std::clock();

	while(ros::ok() && INITIALIZED)
	{
		ros::spinOnce();
		loop_rate.sleep();
		try
		{
			if(Operation_Mode == "LIVE")
			{
				char buf = 0;
				char response[255];
				char message_buffer[255];
				int spot = 0;
				memset(response,0,sizeof response);
				memset(message_buffer,0,sizeof message_buffer);
				int message_size = 0;
				in_message_completed = 0;
				int length = 0;
				int temp_counter = 0;
				
				in_message_started = 0;
				in_message_completed = 0;
				in_message_ready = 0;
				/*while(in_message_ready == 0)
				{
					memset(response,0,sizeof response);
					res = read(sc_device,&response,sizeof(response));
					if(res > 4)
					{
						in_message_ready = 1;
						printf("Read %d bytes\r\n",res);
						if(in_message_started == 0)
						{
							for(int i = 0; i < (res-3); i++)
							{
								if ((response[i]   == 'A') and
								(response[i+1] == 'B') and
								(response[i+2] == '1') and
								(response[i+3] == '2'))
								{
									in_message_started = 1;
									
									
								}
						}
					}
				}*/
				memset(response,0,sizeof response);
				res = read(sc_device,&response,sizeof(response));
				if(res == 32){ in_message_ready = 1; }
				/*if(res == 32)
				{
					for(int i = 0; i < res;i++)
					{
						printf("%c",response[i]);
					}
					printf(", %d bytes\r\n",res);
				}*/
				if(in_message_ready == 1)
				{
					for(int i = 0; i < res; i++)
					{
						if ((response[i]   == 'A') and
							(response[i+1] == 'B') and
							(response[i+2] == '1') and
							(response[i+3] == '2'))
						{
							SonarDistance_1 = hex2dec(response[i+4],response[i+5]);
							SonarDistance_2 = hex2dec(response[i+6],response[i+7]);
							SonarDistance_3 = hex2dec(response[i+8],response[i+9]);
							SonarDistance_4 = hex2dec(response[i+10],response[i+11]);
							SonarDistance_5 = hex2dec(response[i+12],response[i+13]);
							SonarDistance_6 = hex2dec(response[i+14],response[i+15]);
							SonarDistance_7 = hex2dec(response[i+16],response[i+17]);
							SonarDistance_8 = hex2dec(response[i+18],response[i+19]);
							SonarDistance_9 = hex2dec(response[i+20],response[i+21]);
							SonarDistance_10 = hex2dec(response[i+22],response[i+23]);
							SonarDistance_11 = hex2dec(response[i+24],response[i+25]);
							SonarDistance_12 = hex2dec(response[i+26],response[i+27]);
							SonarDistance_13 = hex2dec(response[i+28],response[i+29]);

							in_message_ready = 1;
							in_message_completed = 1;
							printf("Got AB12 v1: %d v2: %d v3: %d v4: %d v5: %d v6: %d v7: %d v8: %d v9: %d v10: %d v11: %d v12: %d v13: %d\r\n",
							SonarDistance_1,SonarDistance_2,SonarDistance_3,SonarDistance_4,SonarDistance_5,SonarDistance_6,SonarDistance_7,
							SonarDistance_8,SonarDistance_9,SonarDistance_10,SonarDistance_11,SonarDistance_12,SonarDistance_13);
							
						}
					}		
					
				}

				if((in_message_ready == 1))
				{
					dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
					start = std::clock();

				
					Sonar_Scan.header.stamp = ros::Time::now();
					Sonar_Scan.header.frame_id = "/laser";
					Sonar_Scan.header.seq = sequence_counter;
					sequence_counter++;
					Sonar_Scan.angle_min = (float)(angle_min-beam_width/2.0)*PI/180.0;
					Sonar_Scan.angle_max = (float)(angle_max+beam_width/2.0)*PI/180.0;
					Sonar_Scan.angle_increment = ((Sonar_Scan.angle_max-Sonar_Scan.angle_min)/(ping_sensor_count*points_per_beam));
					Sonar_Scan.range_min = range_min;
					Sonar_Scan.range_max = range_max;
					Sonar_Scan.scan_time = dtime;
					Sonar_Scan.ranges.resize(ping_sensor_count*points_per_beam);
					
					ping_distances[0] = SonarDistance_1*0.0254;
					ping_distances[1] = SonarDistance_2*0.0254;
					ping_distances[2] = SonarDistance_3*0.0254;
					ping_distances[3] = SonarDistance_4*0.0254;
					ping_distances[4] = SonarDistance_5*0.0254;
					ping_distances[5] = SonarDistance_6*0.0254;
					ping_distances[6] = SonarDistance_7*0.0254;
					ping_distances[7] = SonarDistance_8*0.0254;
					ping_distances[8] = SonarDistance_9*0.0254;
					ping_distances[9] = SonarDistance_10*0.0254;
					ping_distances[10] = SonarDistance_11*0.0254;
					ping_distances[11] = SonarDistance_12*0.0254;
					ping_distances[12] = SonarDistance_13*0.0254;
					
					for(int j = 0; j < ping_sensor_count; j++)
					{
						if (ping_distances[j] < range_min) { ping_distances[j] = range_min; }
						else if (ping_distances[j] > range_max) { ping_distances[j] = range_max; }
						for(int k = j*points_per_beam; k < (j+1)*points_per_beam;k++)
						{
							Sonar_Scan.ranges[k] = ping_distances[j];
						}
						
						
						
						//Sonar_Scan.ranges[j] = 3.5;
					}
					Pub_ICARUS_Sonar_Scan.publish(Sonar_Scan);
					geometry_msgs::Quaternion scan_quat = tf::createQuaternionMsgFromYaw(PI);
					geometry_msgs::TransformStamped scan_trans;
					scan_trans.header.stamp = ros::Time::now();
					scan_trans.header.frame_id = "/base_link";
					scan_trans.child_frame_id = "/laser";
					scan_trans.transform.translation.x = 0.0;
					scan_trans.transform.translation.y = 0.0;
					scan_trans.transform.translation.z = 0.3556;
					scan_trans.transform.rotation = scan_quat;
					scan_broadcaster.sendTransform(scan_trans);
					
					ICARUS_Diagnostic.Diagnostic_Type = NO_ERROR;
					ICARUS_Diagnostic.Level = NO_ERROR;
					ICARUS_Diagnostic.Diagnostic_Message = NO_ERROR;
					Pub_ICARUS_Sonic_Controller_Diagnostic.publish(ICARUS_Diagnostic);
				}
			}
			else if(Operation_Mode=="SIM")
			{
				geometry_msgs::Quaternion scan_quat = tf::createQuaternionMsgFromYaw(90.0*PI/180.0);
				geometry_msgs::TransformStamped scan_trans;
				scan_trans.header.stamp = ros::Time::now();
				scan_trans.header.frame_id = "/base_link";
				scan_trans.child_frame_id = "/laser";
				scan_trans.transform.translation.x = 0.0;
				scan_trans.transform.translation.y = 0.0;
				scan_trans.transform.translation.z = 0.3556;
				scan_trans.transform.rotation = scan_quat;
				scan_broadcaster.sendTransform(scan_trans);
				//printf("Published laser -> base_link tf\r\n");
			}

		}
		catch(const std::exception& ex)
		{
			ROS_INFO("ERROR:%s",ex.what());
			if(Operation_Mode=="LIVE")
			{
				close(sc_device);
			}
			ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
			ICARUS_Diagnostic.Level = FATAL;
			ICARUS_Diagnostic.Diagnostic_Message = GENERAL_ERROR;
			Pub_ICARUS_Sonic_Controller_Diagnostic.publish(ICARUS_Diagnostic);	
		}
	}
	if(Operation_Mode=="LIVE")
	{
		close(sc_device);
	}

}

int hex2dec(char hexdigit)
{
	int temp = 0;
	switch (hexdigit)
	{
		case '0':
			temp = hexdigit - 48;
			break;
		case '1':
			temp = hexdigit - 48;
			break;
		case '2':
			temp = hexdigit - 48;
			break;
		case '3':
			temp = hexdigit - 48;
			break;
		case '4':
			temp = hexdigit - 48;
			break;
		case '5':
			temp = hexdigit - 48;
			break;
		case '6':
			temp = hexdigit - 48;
			break;
		case '7':
			temp = hexdigit - 48;
			break;
		case '8':
			temp = hexdigit - 48;
			break;
		case '9':
			temp = hexdigit - 48;
			break;
		case 'A':
			temp = hexdigit - 55;
			break;
		case 'B':
			temp = hexdigit - 55;
			break;
		case 'C':
			temp = hexdigit - 55;
			break;
		case 'D':
			temp = hexdigit - 55;
			break;
		case 'E':
			temp = hexdigit - 55;
			break;
		case 'F':
			temp = hexdigit - 55;
			break;
		default:
			temp = 0;
			break;
	}
	return temp;
}
int hex2dec(char hexdigit1,char hexdigit2)
{
	int temp = 0;
	temp = hex2dec(hexdigit1)*16 + hex2dec(hexdigit2);
	return temp;
}
