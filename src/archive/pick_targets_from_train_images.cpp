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
#include <fstream>
#include <string>
#include <math.h>
#include <dirent.h>
#include <iomanip>

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image w/ Overlay";
string image_folder;
string capture_date;
string picked_date;
string target_filename;
ofstream target_file;
string root_directory;

//target_file.open(
int targets[2][2];
int target_counter = 0;
image_transport::Publisher pub;
cv_bridge::CvImagePtr cv_ptr;
  cv::Mat image;

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
            if (target_counter < 2)
            {
              targets[target_counter][0] = x;
              targets[target_counter][1] = y;
              
              
              target_counter++;
            }
            
          }
          catch(runtime_error& ex)
          {
            ROS_INFO("Exception converting image to PNG format: %s\n", ex.what());
          }
          
     }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
        targets[0][0] = -1;
        targets[0][1] = -1;
        targets[1][0] = -1;
        targets[1][1] = -1;
        target_counter = 2;
     }
     else if  ( event == cv::EVENT_MBUTTONDOWN )
     {
      }
          //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     else if ( event == cv::EVENT_MOUSEMOVE )
     {
          //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }
}

int main(int argc, char **argv)
{



  ros::init(argc, argv, "image_acqusition");
  ros::NodeHandle nh;
  //nh.getParam("target_count",target_count);
  nh.getParam("root_directory",root_directory);
  nh.getParam("capture_date",capture_date);
  nh.getParam("picked_date",picked_date);
  ros::Rate loop_rate(10);
  cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
  cv::setMouseCallback(WINDOW,mouseCallBack,NULL);
  image_folder = root_directory + "media/imagesamples-" + capture_date + "/";
  target_filename = root_directory + "target_files/" + picked_date + ".csv";
  target_file.open(target_filename.c_str());
  vector<string> image_names = vector<string>();
  
  string tempstr2;
  get_dircontents (image_folder,image_names);
  int image_count = image_names.size()-2;
  for (unsigned int i = 0;i < image_names.size();i++) 
  {
    stringstream sw;
    string tempstr;
    sw << image_names[i];
    tempstr = sw.str();

    if (tempstr.length() > 4)
    {
      ROS_INFO("Processing Image %d/%d with name:%s\n",i,image_count,tempstr.c_str());
      tempstr2 = image_folder + "/" + image_names[i];
      //sprintf(tempstr2,"%s/%s",image_folder,image_names[i].c_str());
    
      try
      {
        image = imread(tempstr2.c_str(), CV_LOAD_IMAGE_COLOR);
        
        imshow(WINDOW, image ); 
        cv::waitKey(0); 
        cv::circle(image,Point(targets[0][0],targets[0][1]),5,cv::Scalar(0,0,255),CV_FILLED,8,0);
        cv::circle(image,Point(targets[1][0],targets[1][1]),5,cv::Scalar(0,0,255),CV_FILLED,8,0);
         target_file << image_names[i].c_str() << "," << targets[0][0] << "," << targets[0][1] << "," << targets[1][0] << "," << targets[1][1] << endl;

        ROS_INFO("Target1 x:%d y:%d\n", targets[0][0],targets[0][1]); 
        ROS_INFO("Target2 x:%d y:%d\n", targets[1][0],targets[1][1]); 
        imshow(WINDOW, image ); 
        cv::waitKey(0); 
        target_counter = 0;
        targets[0][0] = -1;
        targets[0][1] = -1;
        targets[1][0] = -1;
        targets[1][1] = -1;
      }
      catch(runtime_error& ex)
      {
        ROS_INFO("Exception: %s\n", ex.what());
      }
    }
    
  }
  /*while(ros::ok())
  {
    
    ros::spinOnce();
    loop_rate.sleep();
  }*/
  cv::destroyWindow(WINDOW);


  
  
}
