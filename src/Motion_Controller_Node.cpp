//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
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
#include "icarus_rover_rc/Definitions.h"
#include "icarus_rover_rc/ICARUS_Probe_Status.h"
#include "icarus_rover_rc/ICARUS_Probe_Command.h"
#include "icarus_rover_rc/ICARUS_Diagnostic.h"

using namespace std;

int Extended_Switch = 0;
int Retracted_Switch = 0;
int Forward_Timeout = 0;
int Reverse_Timeout = 0;
int Recharge_Command = RECHARGE_WAIT;
double forward_timer = 0.0;
double reverse_timer = 0.0;

//Communication Variables
int token_index = 0;
string MC_Device = "";
int Baud_Rate = -1;
int in_message_started = 0;
int in_message_completed = 0;

//Position Variables
double Pose_X;
double Pose_Y;
double Heading;

//Motion Variables
int Steer_Command;
int Drive_Command;
int armed_state;

//Other Variables
int Probe_State = PROBE_RETRACTED;
int Probe_Error = NO_ERROR;
int DEBUG_MODE;
double dtime = 0.0;
int temp1;


void ICARUS_Probe_Command_Callback(const icarus_rover_rc::ICARUS_Probe_Command::ConstPtr& msg)  //Process ICARUS Probe Command Message
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  Recharge_Command = 0;//msg.Charge_Command;
  
}
void ICARUS_Rover_Control_Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//Assume Joy values are float, -1.0 to 1.0
  Steer_Command = (int)(joy->axes[0]+0.5)*255.0;
  Drive_Command = (int)(joy->axes[1]+0.5)*255.0;
  
}
void Recharge_FSM(double dtime)
{
  switch (Probe_State)
  {
    case PROBE_MOVING_FORWARD:
      forward_timer += dtime;
      if (Extended_Switch == true)
      { 
        Probe_State = PROBE_EXTENDED; 
        Probe_Error = NO_ERROR;
      }
      //servoWrite(WINCH_MOTOR_PIN, WINCH_MOTOR_FORWARD);
      break;
    case PROBE_MOVING_REVERSE:
      reverse_timer += dtime;
      if (Retracted_Switch == true) 
      { 
        Probe_State = PROBE_RETRACTED; 
      }
      //servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_REVERSE);
      break;
    case PROBE_EXTENDED:
      forward_timer = 0.0;
      reverse_timer = 0.0;
      if (Recharge_Command == RECHARGE_STOP)
      {
        Probe_State = PROBE_MOVING_REVERSE;
      }
      //servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
    case PROBE_RETRACTED:
      forward_timer = 0.0;
      reverse_timer = 0.0;
      if (Recharge_Command == RECHARGE_START) 
      { 
        Probe_State = PROBE_MOVING_FORWARD; 
        Probe_Error = NO_ERROR;
      }
      //servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
    default:
      //servoWrite(WINCH_MOTOR_PIN,WINCH_MOTOR_NEUTRAL);
      break;
  }
  if (forward_timer > FORWARD_TIME_LIMIT){ Probe_Error = EXTENSION_ERROR; }
  if (reverse_timer > REVERSE_TIME_LIMIT){ Probe_State = RETRACTION_ERROR; }
}
int main(int argc, char **argv)
{
  int INITIALIZED = 0;
  ros::init(argc, argv, "Motion_Controller");
  ros::NodeHandle nh("~");
  //nh.getParam("target_count",target_count);
  nh.getParam("mc_device",MC_Device);
  nh.getParam("baudrate",Baud_Rate);
  nh.getParam("DEBUG_MODE",DEBUG_MODE);
  int mc_device;
  struct termios oldtio,newtio;
	int res;
  
  mc_device= open(MC_Device.c_str(),O_RDWR | O_NOCTTY );

  if (mc_device  == -1)
  {
    printf("ERROR: UNABLE TO OPEN MOTION CONTROLLER PORT ON %s.",MC_Device.c_str());
  }
  else
  {
    if(1)
    {
      INITIALIZED = 1;
      tcgetattr(mc_device,&oldtio);
	    bzero(&newtio, sizeof(newtio));
	    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
      //newtio.c_cflag &= ~(IXON | IXOFF | IXANY);
	    newtio.c_iflag = IGNPAR;
	    newtio.c_oflag = 0;
	    newtio.c_lflag = 0;
	    newtio.c_cc[VTIME]    = 1;
	    newtio.c_cc[VMIN]     = 5;
                  // 0.5 seconds read timeout


	    tcflush(mc_device, TCIFLUSH);
	    tcsetattr(mc_device,TCSANOW,&newtio);
    }
    else
    {
struct termios tty;
struct termios tty_old;
memset (&tty, 0, sizeof tty);

/* Error Handling */
if ( tcgetattr ( mc_device, &tty ) != 0 ) {
   std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
}

/* Save old tty parameters */
tty_old = tty;

/* Set Baud Rate */
cfsetospeed (&tty, (speed_t)B9600);
cfsetispeed (&tty, (speed_t)B9600);

/* Setting other Port Stuff */
tty.c_cflag     &=  ~PARENB;            // Make 8n1
tty.c_cflag     &=  ~CSTOPB;
tty.c_cflag     &=  ~CSIZE;
tty.c_cflag     |=  CS8;

tty.c_cflag     &=  ~CRTSCTS;           // no flow control
tty.c_cc[VMIN]   =  1;                  // read doesn't block
tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

/* Make raw */
cfmakeraw(&tty);

/* Flush Port, then applies attributes */
tcflush( mc_device, TCIFLUSH );
if ( tcsetattr ( mc_device, TCSANOW, &tty ) != 0) {
   std::cout << "Error " << errno << " from tcsetattr" << std::endl;
}}
}  

  ros::Subscriber Sub_ICARUS_Probe_Command_Callback = nh.subscribe("ICARUS_Probe_Command", 1000, ICARUS_Probe_Command_Callback);
  ros::Publisher Pub_ICARUS_Probe_Status = nh.advertise<icarus_rover_rc::ICARUS_Probe_Status>("ICARUS_Probe_Status", 1000);  
  ros::Publisher Pub_ICARUS_Motion_Controller_Diagnostic = nh.advertise<icarus_rover_rc::ICARUS_Diagnostic>("ICARUS_Motion_Controller_Diagnostic",1000);
  ros::Subscriber Pub_Rover_Control = nh.subscribe<sensor_msgs::Joy>("ICARUS_Rover_Control",1000,ICARUS_Rover_Control_Callback);
  ros::Publisher Pub_ICARUS_Rover_Pose = nh.advertise<geometry_msgs::Pose2D>("ICARUS_Rover_Pose",1000);
  ros::Rate loop_rate(100);
	std::clock_t    start;
  ::icarus_rover_rc::ICARUS_Probe_Status Probe_Status;
  ::icarus_rover_rc::ICARUS_Diagnostic ICARUS_Diagnostic;
  geometry_msgs::Pose2D Rover_Pose;
  ICARUS_Diagnostic.header.frame_id = "ICARUS_Motion_Controller_Diagnostic";
  ICARUS_Diagnostic.System = ROVER;
  ICARUS_Diagnostic.SubSystem = ROBOT_CONTROLLER;
  ICARUS_Diagnostic.Component = MOTION_CONTROLLER_NODE;
  
  temp1 = 0;
	while( ros::ok() && INITIALIZED)
	{
    
		start = std::clock();
	  	ros::spinOnce();
	  	loop_rate.sleep();
	  	if(armed_state == ARMED)
	  	{
			if(DEBUG_MODE >= INFORMATION) { printf("Armed State: ARMED\r\n"); }
		}
		else if(armed_state == DISARMED)
		{
			if(DEBUG_MODE >= INFORMATION) { printf("Armed State: DISARMED\r\n"); }
		}
		else
		{
			printf("Armed Sttate: UNKNOWN!!!\r\n");
		}
	  	try
	  {
	    int wr;
      temp1++;
      if (temp1 > 180) { temp1 = 0; }
      
      char cmd[255];
      memset(cmd,'\0',sizeof cmd);
	Steer_Command = STEER_SERVO_CENTER;
	Drive_Command = DRIVE_MOTOR_NEUTRAL;
      sprintf(cmd,"$NAV,%d,%d,*\r\n",(int)Steer_Command,(int)Drive_Command);
	if (DEBUG_MODE == 1) { printf("$NAV,%d,%d,*\r\n",(int)Steer_Command,(int)Drive_Command);}
      //printf();
      wr = write(mc_device,cmd,sizeof(cmd)-1);
      char buf = '\0';
      char response[255];
      int spot = 0;
    
      memset(response,'\0',sizeof response);
      //res = read(mc_device,&buf,1);
      //printf("x%c",buf);
      
      do
      {
        
        res = read(mc_device,&buf,1);
        if (buf == '$') 
        {
          in_message_started = 1;
          in_message_completed = 0;
        }
        if(in_message_started == 1)
        {
          sprintf(&response[spot],"%c",buf);
          spot += res;
        }
      } while(buf != '*' && res > 0);
      in_message_started = 0;
      in_message_completed = 1;
      string in_message(response);
	if (DEBUG_MODE == 1) { printf("%s\r\n",in_message.c_str()); }	
	if (in_message.compare(0,4,"$POS") == 0)
      	{
        	istringstream ss(in_message);
       	 	string token;
        	token_index = 0;
        	while(getline(ss,token,','))
		{
          		std::string::size_type sz;
         		if(token_index == 1) { Pose_X = atof(token.c_str()); }
          		if(token_index == 2) { Pose_Y = atof(token.c_str()); }
          		if(token_index == 3) { Heading = atof(token.c_str()); }
          		token_index++;
        	}
        	ICARUS_Diagnostic.Diagnostic_Type = COMMUNICATIONS;
		ICARUS_Diagnostic.Level = DEBUG;
		ICARUS_Diagnostic.Diagnostic_Message = NO_ERROR;
	//ICARUS_Diagnostic.Description =  "";
		if (DEBUG_MODE == 1)  { cout << setprecision(8) << "X: " << Pose_X << " Y: " << Pose_Y << " Heading: " << Heading << " Len: " << spot << endl; }
        	Rover_Pose.x = Pose_X;
		Rover_Pose.y = Pose_Y;
		Rover_Pose.theta = Heading;
		Pub_ICARUS_Rover_Pose.publish(Rover_Pose);
      	}
      	else if(in_message.compare(0,4,"$STA") == 0)
    	{
		if(in_message.compare(5,3,"ARM") == 0)
		{
	    		istringstream ss(in_message);
	    		string token;
            		token_index = 0;
	    		while(getline(ss,token,','))
			{
	      			std::string::size_type sz;
	      			if(token_index == 2) { armed_state = atoi(token.c_str()); }
				token_index++;
	    		}
	    		
		}
    	}
	else
	{
		ICARUS_Diagnostic.header.stamp = ros::Time::now();
		ICARUS_Diagnostic.Diagnostic_Type = COMMUNICATIONS;
		ICARUS_Diagnostic.Level = CAUTION;
		ICARUS_Diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
	}
/*
int n = 0,
    spot = 0;
char buf = '\0';


char response[255];
memset(response, '\0', sizeof response);

do {
   n = read( mc_device, &buf, 1 );
   if (n > 0) { printf("%c",buf); }
   sprintf( &response[spot], "%c", buf );
   spot += n;
} while( buf != '\r' && n > 0);
if (n < 0) {
   //std::cout << "Error reading: " << strerror(errno) << std::endl;
}
else if (n == 0) {
    std::cout << "Read nothing!" << std::endl;
}
else {
    std::cout << "Response: " << response << std::endl;
}*/
	    dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
      /*if (digitalRead(EXTENDED_SWITCH_PIN)==0)
      {
        Extended_Switch = true;
      }
      else
      {
        Extended_Switch = false;
      }
      if (digitalRead(RETRACTED_SWITCH_PIN)==0)
      {
        Retracted_Switch = true;
      }
      else
      {
        Retracted_Switch = false;
      }*/

      //Recharge_FSM(dtime);
      Probe_Status.header.stamp = ros::Time::now();
      Probe_Status.header.frame_id = "Probe_Status";
      Probe_Status.Probe_State = Probe_State;
      Probe_Status.Extended_Switch = Extended_Switch;
      Probe_Status.Retracted_Switch = Retracted_Switch;  
      Probe_Status.Probe_Error = Probe_Error;

    
	    Pub_ICARUS_Probe_Status.publish(Probe_Status);  

      //ICARUS Diagnostics Publisher
      ICARUS_Diagnostic.header.stamp = ros::Time::now();
      Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
	  }
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
      close(mc_device);
      
      //ICARUS Diagnostics Publisher
      ICARUS_Diagnostic.header.stamp = ros::Time::now();
      ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
      ICARUS_Diagnostic.Level = FATAL;
      ICARUS_Diagnostic.Diagnostic_Message = GENERAL_ERROR;
      ICARUS_Diagnostic.Description = ex.what();
      Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
	  }
  }
  close(mc_device);
  ICARUS_Diagnostic.header.stamp = ros::Time::now();
  ICARUS_Diagnostic.Diagnostic_Type = GENERAL_ERROR;
  ICARUS_Diagnostic.Level = SEVERE;
  ICARUS_Diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
  ICARUS_Diagnostic.Description = "Node Closed.";
  Pub_ICARUS_Motion_Controller_Diagnostic.publish(ICARUS_Diagnostic);
  
}
