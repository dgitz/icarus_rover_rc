#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
//#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "truck_spotting_phase1a/Vehicle.h"
#include "applanix_msgs/NavigationSolution.h"

//User Input Values
int SHOW_IMAGES = 1;

namespace enc = sensor_msgs::image_encodings;
using namespace std;

static const char WINDOW[] = "Topview w/ Overlay";
float GPS_fixedA[2][5] = {{40.851967,40.851607,40.851066,40.851119,40.851495},{-89.571279,-89.568781,-89.570881,-89.568766,-89.569697}};
int Pix_fixedA[2][5] = {{235,1450,427,1451,1005},{12,278,625,594,351}};
float inch_fixedB[2][4] = {{0.0,204.0,0.0,204.0},{0.0,0.0,85.0,85.0}};
int Pix_fixedB[2][5] = {{992,1020,992,1021,1006},{345,345,355,357,350}};


Vehicle myVehicle(1);

struct gps_coordinate{
  float latitude;
  float longitude;
};
struct pixel_coordinate{
  int x;
  int y;
}; 
struct inch_coordinate{
  int x;
  int y;
};
image_transport::Publisher pub;  

pixel_coordinate gps2pixel(gps_coordinate mygps)
{
  pixel_coordinate mypixel;
  float x0 = GPS_fixedA[1][0];
  float x1 = GPS_fixedA[1][3];
  float y0 = (float)Pix_fixedA[0][0];
  float y1 = (float)Pix_fixedA[0][3];
  int y = int((y0 + (y1-y0)*(mygps.longitude-x0)/(x1-x0)));  
  mypixel.x = y;

  x0 = GPS_fixedA[0][0];
  x1 = GPS_fixedA[0][3];
  y0 = (float)Pix_fixedA[1][0];
  y1 = (float)Pix_fixedA[1][3];
  y = int((y0 + (y1-y0)*(mygps.latitude-x0)/(x1-x0)));  
  mypixel.y = y;
  //mypixel.x = (int)(y0+(y1-y0)*(mygps.longitude));
  
  return mypixel;

}
gps_coordinate pixel2gps(pixel_coordinate mypixel)
{
  gps_coordinate mygps;
  float y0 = GPS_fixedA[1][0];
  float y1 = GPS_fixedA[1][3];
  float x0 = (float)Pix_fixedA[0][0];
  float x1 = (float)Pix_fixedA[0][3];
  float y = (y0 + (y1-y0)*(mypixel.x-x0)/(x1-x0));  
  mygps.longitude = y;

  y0 = GPS_fixedA[0][0];
  y1 = GPS_fixedA[0][3];
  x0 = (float)Pix_fixedA[1][0];
  x1 = (float)Pix_fixedA[1][3];
  y = (y0 + (y1-y0)*(mypixel.y-x0)/(x1-x0));  
  mygps.latitude = y;
  return mygps;
}
pixel_coordinate inch2pixel(inch_coordinate myinch)
{
  pixel_coordinate mypixel;
  float x0 = inch_fixedB[1][0];
  float x1 = inch_fixedB[1][3];
  float y0 = (float)Pix_fixedB[0][0];
  float y1 = (float)Pix_fixedB[0][3];
  int y = int((y0 + (y1-y0)*(myinch.x-x0)/(x1-x0)));  
  mypixel.x = y;

  x0 = inch_fixedB[0][0];
  x1 = inch_fixedB[0][3];
  y0 = (float)Pix_fixedB[1][0];
  y1 = (float)Pix_fixedB[1][3];
  y = int((y0 + (y1-y0)*(myinch.y-x0)/(x1-x0)));  
  mypixel.y = y;
  return mypixel;
}
inch_coordinate pixel2inch(pixel_coordinate mypixel)
{
  inch_coordinate myinch;
  float y0 = inch_fixedB[1][0];
  float y1 = inch_fixedB[1][3];
  float x0 = (float)Pix_fixedB[0][0];
  float x1 = (float)Pix_fixedB[0][3];
  int y = int((y0 + (y1-y0)*(mypixel.x-x0)/(x1-x0)));  
  myinch.x = y;

  y0 = inch_fixedB[0][0];
  y1 = inch_fixedB[0][3];
  x0 = (float)Pix_fixedB[1][0];
  x1 = (float)Pix_fixedB[1][3];
  y = int((y0 + (y1-y0)*(mypixel.y-x0)/(x1-x0)));  
  myinch.y = y;
  return myinch;
}

void mouseCallBack(int event, int x, int y, int flags, void* userdata)
{
  gps_coordinate newgps;
  pixel_coordinate newpixel;
  pixel_coordinate newpixel2;
  newpixel.x = x;
  newpixel.y = y;
  newgps = pixel2gps(newpixel);
  newpixel2 = gps2pixel(newgps);
  inch_coordinate newinch;
  newinch = pixel2inch(newpixel);
  cout << "x:" << x << " y:" << y <<  " inx:" << newinch.x << " iny:" << newinch.y << endl;//" cx:" << newpixel2.x << " cy:" << newpixel2.y << " Long:" << setprecision(10) << newgps.longitude << " Lat:" << setprecision(10) << newgps.latitude << endl;
}

void orientationCallBack(const applanix_msgs::NavigationSolution& msg)
{
    myVehicle.Set_Orientation(msg.roll,msg.pitch,msg.heading);
}
void odomCallBack(const nav_msgs::Odometry& msg)
{
    myVehicle.Set_Position(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z);
    myVehicle.VehiclePose = myVehicle.Get_Pose();
    myVehicle.VehicleBodyPoints = myVehicle.Get_BodyPoints();
   

    if (SHOW_IMAGES)
    {
      //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
          //Always copy, returning a mutable CvImage
          //OpenCV expects color images to use BGR channel order.
          //cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
          //if there is an error during conversion, display it
          ROS_ERROR("ROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
          return;
      }

	    cv::Mat satellite_view;
      satellite_view = cv::imread("/home/gitzd/catkin_ws/src/truck_spotting_phase1a/media/satellite_fixed.png");
      if (!satellite_view.data)
      {
        cout << "Couldn't find/read image." << endl;
      }  
      else
      {  
        pixel_coordinate pix_coord_vehcenter;
        inch_coordinate in_coord_vehcenter;
        in_coord_vehcenter.x = myVehicle.VehiclePose.x;
        in_coord_vehcenter.y = myVehicle.VehiclePose.y;
        pix_coord_vehcenter = inch2pixel(in_coord_vehcenter);

        pixel_coordinate pix_coord_veh_FL;
        inch_coordinate in_coord_veh_FL;
        in_coord_veh_FL.x = myVehicle.VehicleBodyPoints.FL_x;
        in_coord_veh_FL.y = myVehicle.VehicleBodyPoints.FL_y;
        pix_coord_veh_FL = inch2pixel(in_coord_veh_FL);
        cout << in_coord_veh_FL.y << "," << setprecision(6) << sin(45*3.14/180.0) << endl;
        //pix_coord_veh_FL.x = pix_coord_veh_FL.x * cos(myVehicle.Get_Heading()*3.14/180.0);
        //pix_coord_veh_FL.y = pix_coord_veh_FL.y * sin(myVehicle.Get_Heading()*3.14/180.0);
        pixel_coordinate pix_coord_veh_FR;
        inch_coordinate in_coord_veh_FR;
        in_coord_veh_FR.x = myVehicle.VehicleBodyPoints.FR_x;
        in_coord_veh_FR.y = myVehicle.VehicleBodyPoints.FR_y;
        pix_coord_veh_FR = inch2pixel(in_coord_veh_FR);
        
        pixel_coordinate pix_coord_veh_BL;
        inch_coordinate in_coord_veh_BL;
        in_coord_veh_BL.x = myVehicle.VehicleBodyPoints.BL_x;
        in_coord_veh_BL.y = myVehicle.VehicleBodyPoints.BL_y;
        pix_coord_veh_BL = inch2pixel(in_coord_veh_BL);
  
        pixel_coordinate pix_coord_veh_BR;
        inch_coordinate in_coord_veh_BR;
        in_coord_veh_BR.x = myVehicle.VehicleBodyPoints.BR_x;
        in_coord_veh_BR.y = myVehicle.VehicleBodyPoints.BR_y;
        pix_coord_veh_BR = inch2pixel(in_coord_veh_BR);

        cv::circle(satellite_view,cv::Point(pix_coord_vehcenter.x,pix_coord_vehcenter.y),5,cv::Scalar( 0, 0, 255 ),1,8,0);
        //cv::line(satellite_view,cv::Point(pix_coord_veh_FL.x,pix_coord_veh_FL.y),cv::Point(pix_coord_veh_FR.x,pix_coord_veh_FR.y),cv::Scalar(0,255,255),.5,CV_AA);
        //cv::line(satellite_view,cv::Point(pix_coord_veh_FR.x,pix_coord_veh_FR.y),cv::Point(pix_coord_veh_BR.x,pix_coord_veh_BR.y),cv::Scalar(0,255,255),.5,CV_AA);
        //cv::line(satellite_view,cv::Point(pix_coord_veh_BR.x,pix_coord_veh_BR.y),cv::Point(pix_coord_veh_BL.x,pix_coord_veh_BL.y),cv::Scalar(0,255,255),.5,CV_AA);
        cv::line(satellite_view,cv::Point(pix_coord_veh_BL.x,pix_coord_veh_BL.y),cv::Point(pix_coord_veh_FL.x,pix_coord_veh_FL.y),cv::Scalar(0,255,255),.5,CV_AA);
        cv::imshow(WINDOW, satellite_view);   
        cv::setMouseCallback(WINDOW,mouseCallBack,NULL); 
        cv::moveWindow(WINDOW,0,0);
        cv::waitKey(3);
      }
    }  
  //pub.publish(cv_ptr->toImageMsg());
}
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "topview");
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  if (SHOW_IMAGES) { cv::namedWindow(WINDOW);}
  //ros::Subscriber gps = node.subscribe("applanix/gps_fix", 10, &gpsCallBack);
  ros::Subscriber odom = node.subscribe("applanix/gps_odom",10,&odomCallBack);
  ros::Subscriber orientation = node.subscribe("applanix/nav",10,&orientationCallBack);
  if (SHOW_IMAGES) {cv::destroyWindow(WINDOW);
  pub = it.advertise("topview/final_image", 1);}
  ros::spin();
  ROS_INFO("ROSOpenCV::main.cpp::No");
  
}
