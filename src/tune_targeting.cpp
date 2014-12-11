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
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
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
string image_filename;
string image_filepath;
string capture_date;
string picked_date;
string target_filename;
fstream target_file;
string root_directory;
string template_image_name;
string template_image_path;

//target_file.open(
int targets[2][2];
int target_counter = 0;
image_transport::Publisher pub;
cv_bridge::CvImagePtr cv_ptr;
  cv::Mat sensor_image;
cv::Mat template_image;

int main(int argc, char **argv)
{



  ros::init(argc, argv, "target_tuning");
  ros::NodeHandle nh;
  //nh.getParam("target_count",target_count);
  nh.getParam("root_directory",root_directory);
  nh.getParam("capture_date",capture_date);
  nh.getParam("picked_date",picked_date);
  nh.getParam("template_image_name",template_image_name);
  template_image_path = root_directory + "media/Templates/" + template_image_name + ".png";
  template_image = imread(template_image_path.c_str(),CV_LOAD_IMAGE_GRAYSCALE);
  
  ros::Rate loop_rate(10);
  cv::namedWindow("Good Matches", CV_WINDOW_AUTOSIZE);
  cv::waitKey(1);
  image_folder = root_directory + "media/imagesamples-" + capture_date + "/";
  target_filename = root_directory + "target_files/" + picked_date + ".csv";
  target_file.open(target_filename.c_str(), fstream::in);
  vector<string> image_names = vector<string>();
  string tempstr;
  int image_count = image_names.size()-2;
  int minHessian = 400;
  cv::SurfFeatureDetector detector(minHessian);
  vector<KeyPoint> template_keypoints,sensor_keypoints;
  detector.detect(template_image,template_keypoints);
  SurfDescriptorExtractor extractor;
  cv::Mat template_descriptor,sensor_descriptor;
  extractor.compute(template_image,template_keypoints,template_descriptor);
  cv::FlannBasedMatcher matcher;
  vector<DMatch>matches;
  while(getline(target_file,tempstr))
  {
   
    image_filename = tempstr.substr(0,tempstr.find(","));
    ROS_INFO("%s",image_filename.c_str());
    image_filepath = image_folder + image_filename;
    sensor_image = imread(image_filepath.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    detector.detect(sensor_image,sensor_keypoints);
    extractor.compute(sensor_image,sensor_keypoints,sensor_descriptor);
    matcher.match(template_descriptor,sensor_descriptor,matches);
    double max_dist = 0;
    double min_dist = 100;
    for( int i = 0; i < template_descriptor.rows; i++ )
    { 
      double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }

    ROS_INFO("-- Max dist : %f \n", max_dist );
    ROS_INFO("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;

    for( int i = 0; i < template_descriptor.rows; i++ )
    { if( matches[i].distance <= max(2*min_dist, 0.02) )
      { good_matches.push_back( matches[i]); }
    }

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( template_image, template_keypoints, sensor_image, sensor_keypoints,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Show detected matches
    imshow( "Good Matches", img_matches );    
    //imshow(WINDOW, image ); 
        cv::waitKey(0);
    
    
    
    
  }

/*  for (unsigned int i = 0;i < image_names.size();i++) 
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
*/
  /*while(ros::ok())
  {
    
    ros::spinOnce();
    loop_rate.sleep();
  }*/

  cv::destroyWindow(WINDOW);


  
  
}
