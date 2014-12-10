//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>


//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image w/ Overlay";
char image_filename[512];
static const char capture_date[] = "7Dec2014";

image_transport::Publisher pub;
cv_bridge::CvImagePtr cv_ptr;

int image_counter = 0;


  

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
	
	

    cv::imshow(WINDOW, cv_ptr->image);
    
    cv::waitKey(3);
        //pub.publish(cv_ptr->toImageMsg());
}
  
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
void mouseCallBack(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          image_counter++;
          sprintf(image_filename,"/home/gitzd/catkin_ws/src/icarus_rover_rc/media/imagesamples-%s/Image%d.png",capture_date,image_counter);
          ROS_INFO("Saving Image with file: %s",image_filename);
          try
          {
            cv::imwrite(image_filename, cv_ptr->image);
          }
          catch(runtime_error& ex)
          {
            ROS_INFO("Exception converting image to PNG format: %s\n", ex.what());
          }
          
     }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
          //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == cv::EVENT_MBUTTONDOWN )
     {
          //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == cv::EVENT_MOUSEMOVE )
     {
          //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }
}

int main(int argc, char **argv)
{

  int target_count = 0;
  ros::init(argc, argv, "image_acqusition");
  ros::NodeHandle nh;
  nh.getParam("target_count", target_count);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
  //nh.getParam("target_count",target_count);
  ros::Rate loop_rate(10);
  cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
  cv::setMouseCallback(WINDOW,mouseCallBack,NULL);
  while(ros::ok())
  {
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  cv::destroyWindow(WINDOW);


  
  
}
