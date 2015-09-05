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
//#include "icarus_rover_rc/ICARUS_Diagnostic.h"

using namespace std;

 
double dtime = 0.0;
int sonic_node_rate = 0.0;
int ping_sensor_count = 0;
int DEBUG_MODE = 0;
int angle_min,angle_max,range_min,range_max,angle_increment;

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
int hex2dec(char);
int hex2dec(char,char);


int main(int argc, char** argv)
{
	ros::init(argc,argv,"Sonic_Controller");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(100);
	int INITIALIZED = 0;

  	//nh.getParam("target_count",target_count);
  	nh.getParam("sc_device",SC_Device);
  	printf("Got sc_device: %s\r\n",SC_Device.c_str());
  	nh.getParam("baudrate",Baud_Rate);
  	printf("Got baudrate: %d\r\n",Baud_Rate);
  	nh.getParam("rate",sonic_node_rate);
  	printf("Got Rate: %d\r\n",sonic_node_rate);
  	nh.getParam("ping_sensor_count",ping_sensor_count);
  	printf("Got Ping Sonar Count: %d\r\n",ping_sensor_count);
  	nh.getParam("DEBUG_MODE",DEBUG_MODE);
  	printf("Got Debug Mode: %d\r\n",DEBUG_MODE);
  	nh.getParam("angle_min_deg",angle_min);
  	nh.getParam("angle_max_deg",angle_max);
  	nh.getParam("angle_increment_deg",angle_increment);
  	nh.getParam("range_min_inch",range_min);
  	nh.getParam("range_max_inch",range_max);
	printf("Min Range: %d Max Range: %d",range_min,range_max);
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
		printf("Serial Socket: %d\r\n",sc_device);
		/*tcgetattr(sc_device,&oldtio);
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
	  	tcsetattr(sc_device,TCSANOW,&newtio);*/
		
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
                  // 0.5 seconds read timeout

				  

	    tcflush(sc_device, TCIFLUSH);
	    tcsetattr(sc_device,TCSANOW,&newtio);
  	}

  	ros::Publisher Pub_ICARUS_Sonar_Scan = nh.advertise<sensor_msgs::LaserScan>("ICARUS_Sonar_Scan",1000);
  	//ros::Publisher Pub_ICARUS_Sonic_Controller_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Sonic_Controller_Diagnostic",1000);
  	//icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
  	//ros::Rate loop_rate(sonic_node_rate);
	std::clock_t    start;
  	sensor_msgs::LaserScan Sonar_Scan;
  	/*ICARUS_Diagnostic.header.frame_id = "ICARUS_Sonic_Controller_Diagnostic";
 	ICARUS_Diagnostic.System = ROVER;
  	ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
  	ICARUS_Diagnostic.Component = SONIC_CONTROLLER_NODE;*/
	//LaserScan Sonar_Scan;
  	start = std::clock();

	while(ros::ok() && INITIALIZED)
	{
		ros::spinOnce();
		loop_rate.sleep();
		try
		{
			char buf = 0;
			char response[255];
			int spot = 0;
			memset(response,0,sizeof response);
			in_message_completed = 0;
			int length = 0;
			int temp_counter = 0;
			
			in_message_started = 0;
			in_message_completed = 0;
			in_message_ready = 0;
			while(!in_message_completed)
			{
				memset(response,0,sizeof response);
				res = read(sc_device,&response,14);
				//printf("Read %d bytes\r\n",res);
				if (res > 0)
				{
					for(int i = 0; i < res; i++)
					{
						//printf("%c\r\n",response[i]);
					}
					for(int i = 0; i < res; i++)
					{
						if ((response[i]   == 'A') and
							(response[i+1] == 'B') and
							(response[i+2] == '1') and
							(response[i+3] == '2'))
						{
							for(int j = i; j < res; j++)
							{
								//printf("%c\r\n",response[j]);
							}
							SonarDistance_1 = hex2dec(response[i+4],response[i+5]);
							SonarDistance_2 = hex2dec(response[i+6],response[i+7]);
							SonarDistance_3 = hex2dec(response[i+8],response[i+9]);
							SonarDistance_4 = hex2dec(response[i+10],response[i+11]);
							SonarDistance_5 = hex2dec(response[i+12],response[i+13]);
							//ping_distances[0] = value1;
							//ping_distances[1] = value2;
							//ping_distances[2] = value3;
							in_message_ready = 1;
							printf("Got AB12 Message with value1: %d value2: %d value3: %d value4: %d value5: %d\r\n",SonarDistance_1,SonarDistance_2,SonarDistance_3,SonarDistance_4,SonarDistance_5);
							
						}
						
					}
				}
				in_message_completed = 1;
			}

			if((in_message_ready == 1) and (in_message_completed == 1))
			{
				dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
				start = std::clock();

			
				Sonar_Scan.header.stamp = ros::Time::now();
				Sonar_Scan.header.frame_id = "laser";
				Sonar_Scan.angle_min = (float)angle_min*3.145/180.0;
				Sonar_Scan.angle_max = (float)angle_max*3.145/180.0;
				Sonar_Scan.angle_increment = angle_increment*3.145/180.0;
				Sonar_Scan.range_min = range_min;
				Sonar_Scan.range_max = range_max;
				Sonar_Scan.scan_time = dtime;
				Sonar_Scan.ranges.resize(ping_sensor_count);
				
				ping_distances[0] = SonarDistance_1;
				ping_distances[1] = SonarDistance_2;
				ping_distances[2] = SonarDistance_3;
				ping_distances[3] = SonarDistance_4;
				ping_distances[4] = SonarDistance_5;
				
				for(int j = 0; j < ping_sensor_count; j++)
				{
					if (ping_distances[j] < range_min) { ping_distances[j] = range_min; }
					else if (ping_distances[j] > range_max) { ping_distances[j] = range_max; }
					
					Sonar_Scan.ranges[j] = ping_distances[j];
					//Sonar_Scan.intensities[j] = ping_distances[j];//+100; 
				}
				printf("Sonar D1: %d D2: %d D3: %d D4: %d D5: %d Count: %d\r\n",ping_distances[0],ping_distances[1],ping_distances[2],ping_distances[3],ping_distances[4],ping_sensor_count);
				Pub_ICARUS_Sonar_Scan.publish(Sonar_Scan);
				
				/*ICARUS_Diagnostic.header.stamp = ros::Time::now();
				ICARUS_Diagnostic.Diagnostic_Type = NO_ERROR;
				ICARUS_Diagnostic.Level = NO_ERROR;
				ICARUS_Diagnostic.Diagnostic_Message = NO_ERROR;
				Pub_ICARUS_Sonic_Controller_Diagnostic.publish(ICARUS_Diagnostic);*/
			}

		}
		catch(const std::exception& ex)
		{
			ROS_INFO("ERROR:%s",ex.what());
			close(sc_device);
			/*ICARUS_Diagnostic.header.stamp = ros::Time::now();
			ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
			ICARUS_Diagnostic.Level = FATAL;
			ICARUS_Diagnostic.Diagnostic_Message = GENERAL_ERROR;
			Pub_ICARUS_Sonic_Controller_Diagnostic.publish(ICARUS_Diagnostic);	*/
		}
	}
	close(sc_device);

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
