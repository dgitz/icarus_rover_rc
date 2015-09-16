//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.


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
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <time.h> 
#include "icarus_rover_rc/Definitions.h"
#include "icarus_rover_rc/ICARUS_Diagnostic.h"

#define LINE1 254
#define BACKLIGHT_ON 157
#define BACKLIGHT_OFF 128
using namespace std;

//Communication Variables
int token_index = 0;
string DIAG_device = "";
int Baud_Rate = -1;
int Output_Level = 0;
int message_sent = 0;
int message_ready = 0;
char buffer_mcnode[255];
int message_ready_mcnode = 0;
char buffer_scnode[255];
int message_ready_scnode = 0;
double dtime = 0.0;

double time_last_MotionController_Node = 0.0;
double time_last_SonicController_Node = 0.0;
double time_last_Mapping_Node = 0.0;
double time_last_Navigation_Node = 0.0;
double time_last_Evolution_Node = 0.0;

int temp_counter = 0;

void Diagnostic_Callback(const icarus_rover_rc::ICARUS_Diagnostic::ConstPtr& msg)
{
	//if(message_ready == 0)
	//{
		
		if (msg->Level >= Output_Level)
		{
			switch(msg->System)
			{
				case ROVER:
					//printf("System: Rover ");
					break;
				case GROUND_STATION:
					//printf("System: Ground Station ");
					break;
				case REMOTE_CONTROL:
					//printf("System: Remote Control ");
					break;
				default:
					break;
			}
			switch(msg->SubSystem)
			{
				case ENTIRE_SYSTEM:
					///printf("Subystem: Entire System ");
					break;
				case ROBOT_CONTROLLER:
					//printf("Subsystem: Robot Controller ");
					break;
				case MOTION_CONTROLLER:
					//printf("Subsystem: Motion Controller ");
					
					break;
				case SONIC_CONTROLLER:
					//printf("Subsystem: Sonic Controller ");
					
					break;
				default:
					break;
			} 
			switch(msg->Component)
			{
				case ENTIRE_SUBSYSTEM:
					//printf("Component: Entire Subsystem ");
					break;
				case DIAGNOSTIC_NODE:
					//printf("Component: Diagnostic Node ");
					break;
				case NAVIGATION_NODE:
					//printf("Component: Navigation Node ");
					time_last_Navigation_Node = ros::Time::now().toSec();
					break;
				case MOTION_CONTROLLER_NODE:
					//printf("Component: Motion Controller Node ");
					time_last_MotionController_Node = ros::Time::now().toSec();
					sprintf(buffer_mcnode,"$GH13%d%d%d%d%d%d%s*",msg->System,msg->SubSystem,msg->Component,msg->Diagnostic_Type,msg->Level,msg->Diagnostic_Message,msg->Description.c_str());
					message_ready_mcnode = 1;
					break;
				case SONIC_CONTROLLER_NODE:
					//printf("Component: Sonic Controller Node ");
					time_last_SonicController_Node = ros::Time::now().toSec();
					sprintf(buffer_scnode,"$GH13%d%d%d%d%d%d%s*",msg->System,msg->SubSystem,msg->Component,msg->Diagnostic_Type,msg->Level,msg->Diagnostic_Message,msg->Description.c_str());
					message_ready_scnode = 1;
					break;
				case EVOLUTION_NODE:
					//printf("Component: Evolution Node ");
					time_last_Evolution_Node = ros::Time::now().toSec();
					break;
				case TARGETING_NODE:
					//printf("Component: Targeting Node ");
					break;
				case MAPPING_NODE:
					//printf("Component: Mapping Node ");
					time_last_Mapping_Node = ros::Time::now().toSec();
					break;
				default:
					break;
			}

			switch(msg->Diagnostic_Type)
			{
				case NO_ERROR:
					//printf("Type: No Error ");
					break;
				case ELECTRICAL:
					//printf("Type: Electrical ");
					break;
				case SOFTWARE:
					//printf("Type: Software ");
					break;
				case COMMUNICATIONS:
					//printf("Type: Communications ");
					break;
				case SENSORS:
					//printf("Type: Sensors ");
					break;
				case ACTUATORS:
					//printf("Type: Actuators ");
					break;
				case DATA_STORAGE:
					//printf("Type: Data Storage ");
					break;
				case GENERAL_ERROR:
					//printf("Type: General Error ");
					break;
				default:
					break;
			}
		
			switch(msg->Level)
			{
				case NO_ERROR:
					//printf("Level: No Error\r\n");
					break;
				case DEBUG:
					//printf("Level: Debug\r\n");
					break;
				case INFORMATION:
					//printf("Level: Information\r\n");
					break;
				case MINIMAL:
					//printf("Level: Minimal\r\n");
					break;
				case CAUTION:
					//printf("Level: Caution\r\n");
					break;
				case SEVERE:
					//printf("Level: Severe\r\n");
					break;
				case FATAL:
					//printf("Level: Fatal\r\n");
					break;
				default:
					break;
			}
			switch(msg->Diagnostic_Message)   
			{
				case NO_ERROR:
					//printf("Diag: No Error.\r\n");
					break;
				case INITIALIZING:
					//printf("Diag: Initializing.\r\n");
					break;
				case DROPPING_PACKETS:
					//printf("Diag: Dropping Packets.\r\n");
					break;
				case MISSING_HEARTBEATS:
					//printf("Diag: Missing Heartbeats.\r\n");
					break;
				case DEVICE_NOT_AVAILABLE:
					//printf("Diag: Device Not Available.\r\n");
					break;
				case GENERAL_ERROR:
					//printf("Diag: General Error\r\n");
					break;
				default:
					break;
			}
			//printf("%s\r\n",msg->Description.c_str());
		}
	//}
}
int main(int argc, char **argv)
{
  sleep(4); 

  int INITIALIZED = 0;
  ros::init(argc, argv, "Diagnostic Node");
  ros::NodeHandle nh("~");
  //nh.getParam("target_count",target_count);
  nh.getParam("diag_device",DIAG_device);
  //nh.getParam("baudrate",Baud_Rate);
  nh.getParam("Output_Level",Output_Level);
  cout << "Output Level: " <<  Output_Level << endl;
  
  //ros::Subscriber Sub_Diagnostic = nh.subscribe<icarus_rover_rc::ICARUS_Diagnostic>("/Mapping_Node/ICARUS_Mapping_Diagnostic",1000,Diagnostic_Callback);
  ros::Subscriber Sub_Diagnostic2 = nh.subscribe<icarus_rover_rc::ICARUS_Diagnostic>("/Sonic_Controller_Node/ICARUS_Sonic_Controller_Diagnostic",1000,Diagnostic_Callback);
  ros::Subscriber Sub_Diagnostic3 = nh.subscribe<icarus_rover_rc::ICARUS_Diagnostic>("/Motion_Controller_Node/ICARUS_Motion_Controller_Diagnostic",1000,Diagnostic_Callback);
  ros::Rate loop_rate(5);
	std::clock_t    start;
	
	int fd;
	struct ifreq ifr;
	fd = socket(AF_INET,SOCK_DGRAM,0);
	ifr.ifr_addr.sa_family = AF_INET;
	strncpy(ifr.ifr_name,"wlan7",IFNAMSIZ-1);
	ioctl(fd,SIOCGIFADDR,&ifr);
	close(fd);
	char my_ipv4_address[50];
	strcpy(my_ipv4_address,inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
	
	int dc_device;
  	struct termios oldtio,newtio;
  	int buffer_length = 255;
	int message_complete = 0;
 	 int message_started = 0;
  	char message[256];
  	int message_index = 0;
  	int res;
  
  	dc_device= open(DIAG_device.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
  
 	if (dc_device  == -1)
  	{
    		printf("ERROR: UNABLE TO OPEN DIAG CONTROLLER PORT ON %s.",DIAG_device.c_str());
			ros::shutdown();
  	}
  	else
  	{
		printf("Serial Socket: %d\r\n",dc_device);
				
		INITIALIZED = 1;
		tcgetattr(dc_device,&oldtio);
	    bzero(&newtio, sizeof(newtio));
	    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	    newtio.c_iflag = IGNPAR;
	    newtio.c_oflag = 0;
	    newtio.c_lflag = 0;
	    newtio.c_cc[VTIME]    = 1;
	    newtio.c_cc[VMIN]     = 5;
	    tcflush(dc_device, TCIFLUSH);
	    tcsetattr(dc_device,TCSANOW,&newtio);
  	}
	while( ros::ok())
	{
    
		start = std::clock();
		ros::spinOnce();
		loop_rate.sleep();
		
		try
		{
			//printf("%s\n", my_ipv4_address);
			//write(dc_device,my_ipv4_address,1);
			
			//sprintf(buffer,"%s\r\n",my_ipv4_address);
			

			if(message_ready_mcnode == 1)
			{
				write(dc_device,buffer_mcnode,sizeof(buffer_mcnode));
				printf("Sent: %s\r\n",buffer_mcnode);
				message_ready_mcnode = 0;
			}
			if(message_ready_scnode == 1)
			{
				write(dc_device,buffer_scnode,sizeof(buffer_scnode));
				printf("Sent: %s\r\n",buffer_scnode);
				message_ready_scnode = 0;
			}
			/*message_sent = 1;
			char response[256];
			char buf = '\0';
			int spot = 0;
			memset(response,0,sizeof response);
			int in_message_completed = 0;
			int res;
			if(message_sent == 1)
			{
				do
				{
					res = read(dc_device,&buf,1);
					sprintf(&response[spot],"%c",buf);
					spot += res;
				}while((buf != '*') && (res > 0));
				in_message_completed = 1;
				if(in_message_completed == 1) { printf("s: %s",response);}
				message_sent = 0;
			}*/
		}
		catch(const std::exception& ex)
		{
			ROS_INFO("ERROR:%s",ex.what());

		}
	}
  
  
}
/*
//Read a string from the Serial Port
			
			
			
*/
