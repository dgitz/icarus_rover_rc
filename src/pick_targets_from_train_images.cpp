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
#include <dirent.h>


//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image w/ Overlay";
char image_folder[512];
static const char capture_date[] = "7Dec2014";

image_transport::Publisher pub;
cv_bridge::CvImagePtr cv_ptr;


int image_counter = 0;
/*function... might want it in some class?*/
int get_dircontents (string dir, vector<string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

  
  
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
void mouseCallBack(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          try
          {
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
  cv::Mat image;
  ros::init(argc, argv, "image_acqusition");
  ros::NodeHandle nh;
  nh.getParam("target_count", target_count);
  //nh.getParam("target_count",target_count);
  ros::Rate loop_rate(10);
  cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
  //cv::setMouseCallback(WINDOW,mouseCallBack,NULL);
  sprintf(image_folder,"/home/gitzd/catkin_ws/src/icarus_rover_rc/media/imagesamples-%s/",capture_date);
  vector<string> image_names = vector<string>();
  
  char tempstr2[512];
  get_dircontents (image_folder,image_names);
  for (unsigned int i = 0;i < image_names.size();i++) 
  {
    stringstream sw;
    string tempstr;
    sw << image_names[i];
    tempstr = sw.str();
    ROS_INFO("%s\n",tempstr.c_str());
    if (tempstr.length() <= 4)
    {
      
      break;
      
    }
    
    sprintf(tempstr2,"%s/%s",image_folder,image_names[i].c_str());
    try
    {
      image = imread(tempstr2, CV_LOAD_IMAGE_COLOR);
      imshow(WINDOW, image ); 
      cv::waitKey(3);  
    }
    catch(runtime_error& ex)
    {
      ROS_INFO("Exception: %s\n", ex.what());
    }
    
  }
  /*while(ros::ok())
  {
    
    ros::spinOnce();
    loop_rate.sleep();
  }*/
  cv::destroyWindow(WINDOW);


  
  
}
