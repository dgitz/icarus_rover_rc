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
#include <ctime>


//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image w/ Overlay";
int SHOW_IMAGES = 0;
int USE_SURF = 0;
int USE_ORB = 0;
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
cv::Mat raw_image;


int main(int argc, char **argv)
{


  

  ros::init(argc, argv, "target_tuning");
  ros::NodeHandle nh;
  //nh.getParam("target_count",target_count);
  nh.getParam("root_directory",root_directory);
  nh.getParam("capture_date",capture_date);
  nh.getParam("picked_date",picked_date);
  nh.getParam("template_image_name",template_image_name);
  nh.getParam("show_images",SHOW_IMAGES);
  nh.getParam("use_surf",USE_SURF);
  nh.getParam("use_orb",USE_ORB);
  template_image_path = root_directory + "media/Templates/" + template_image_name + ".png";
  raw_image = imread(template_image_path.c_str(),CV_LOAD_IMAGE_GRAYSCALE);
  cv::resize(raw_image,template_image, Size(), 3, 3, INTER_LINEAR );
  ros::Rate loop_rate(1000);
  image_folder = root_directory + "media/imagesamples-" + capture_date + "/";
  target_filename = root_directory + "target_files/" + picked_date + ".csv";
  target_file.open(target_filename.c_str(), fstream::in);
  vector<string> image_names = vector<string>();
  string tempstr;
  int image_count = image_names.size()-2;
  vector<KeyPoint> template_keypoints,sensor_keypoints;
  cv::Mat template_descriptor,sensor_descriptor;

  //SURF Initialization Stuff.  
  int minHessian = 1000;
  cv::SurfFeatureDetector surf_detector(minHessian);
  surf_detector.detect(template_image,template_keypoints);
  SurfDescriptorExtractor surf_extractor;
  cv::FlannBasedMatcher surf_matcher;

  //ORB Initialization Stuff
  int numKeyPoints = 1500;
  float distThreshold = 15.0;

  //instantiate detector, extractor, matcher

  cv::OrbFeatureDetector orb_detector(numKeyPoints);
  orb_detector.detect(template_image,template_keypoints);
  cv::OrbDescriptorExtractor orb_extractor;
  cv::FlannBasedMatcher orb_matcher;
  if (USE_SURF)
  {
    
    surf_extractor.compute(template_image,template_keypoints,template_descriptor);
    
  }
  if (USE_ORB)
  {
    orb_extractor.compute(template_image,template_keypoints,template_descriptor);
    if(template_descriptor.type()!=CV_32F) 
    {
      template_descriptor.convertTo(template_descriptor, CV_32F);
    }
    if(sensor_descriptor.type()!=CV_32F) 
    {
      sensor_descriptor.convertTo(sensor_descriptor, CV_32F);
    }
  }
  
  vector<DMatch>matches;
  std::clock_t    start;
  double etime = 0.0;
  int image_counter = 0;
  while(getline(target_file,tempstr) && ros::ok())
  {
    
    ros::spinOnce();
    loop_rate.sleep();
    image_counter++;
    start = std::clock();
    image_filename = tempstr.substr(0,tempstr.find(","));
    
    image_filepath = image_folder + image_filename;
    sensor_image = imread(image_filepath.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if (USE_SURF)
    {
      surf_detector.detect(sensor_image,sensor_keypoints);
      surf_extractor.compute(sensor_image,sensor_keypoints,sensor_descriptor);
    }
    if (USE_ORB)
    {
      orb_detector.detect(sensor_image,sensor_keypoints);
      orb_extractor.compute(sensor_image,sensor_keypoints,sensor_descriptor);
    }
    ROS_INFO("Processing Image:%s",image_filename.c_str());
    try
    {
      if (USE_SURF)
      {
        surf_matcher.match(template_descriptor,sensor_descriptor,matches);
      }
      if (USE_ORB)
      {
        orb_matcher.add(template_descriptor); 
        ROS_INFO("Got Here.");
        //orb_matcher.match(template_descriptor,sensor_descriptor,matches);
      }
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

      std::vector< DMatch > good_matches;

      for( int i = 0; i < template_descriptor.rows; i++ )
      { 
        if( matches[i].distance < 3*min_dist )
        { 
          good_matches.push_back( matches[i]); 
        }
      }

      //-- Draw only "good" matches
      Mat img_matches;
      if (USE_SURF)
      {
        drawMatches( template_image, template_keypoints, sensor_image, sensor_keypoints,
                   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
      }

      std::vector<Point2f> obj;
      std::vector<Point2f> scene;

      for( int i = 0; i < good_matches.size(); i++ )
      {
        //-- Get the keypoints from the good matches
        obj.push_back( template_keypoints[ good_matches[i].queryIdx ].pt );
        scene.push_back( sensor_keypoints[ good_matches[i].trainIdx ].pt );
      }

      Mat H = findHomography( obj, scene, CV_RANSAC );

      //-- Get the corners from the image_1 ( the object to be "detected" )
      std::vector<Point2f> obj_corners(4);
      obj_corners[0] = cvPoint(0,0); 
      obj_corners[1] = cvPoint( template_image.cols, 0 );
      obj_corners[2] = cvPoint( template_image.cols, template_image.rows ); 
      obj_corners[3] = cvPoint( 0, template_image.rows );
      std::vector<Point2f> scene_corners(4);

      perspectiveTransform( obj_corners, scene_corners, H);

      //-- Draw lines between the corners (the mapped object in the scene - image_2 )
      /*line( img_matches, scene_corners[0] + Point2f( template_image.cols, 0), scene_corners[1] + Point2f( template_image.cols, 0), Scalar(0, 255, 0), 4 );
      line( img_matches, scene_corners[1] + Point2f( template_image.cols, 0), scene_corners[2] + Point2f( template_image.cols, 0), Scalar( 0, 255, 0), 4 );
      line( img_matches, scene_corners[2] + Point2f( template_image.cols, 0), scene_corners[3] + Point2f( template_image.cols, 0), Scalar( 0, 255, 0), 4 );
      line( img_matches, scene_corners[3] + Point2f( template_image.cols, 0), scene_corners[0] + Point2f( template_image.cols, 0), Scalar( 0, 255, 0), 4 );*/
    double centroid_x = 0; 
    double centroid_y = 0;

    for(int i = 0;i<4;i++)
    {
        centroid_x += scene_corners[i].x;
        centroid_y += scene_corners[i].y;
    }

    centroid_x = centroid_x/4;
    centroid_y = centroid_y/4;

    ROS_INFO("P:%f,%f",centroid_x,centroid_y);
    cv::circle(img_matches,Point(centroid_x,centroid_y),5,cv::Scalar(0,0,255),CV_FILLED,8,0);
    //-- Show detected matches
    if (SHOW_IMAGES)
    {
      imshow( "Good Matches & Object detection", img_matches );
      cv::waitKey(1);
    }
      etime += (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
      
    }
    catch(const std::exception& ex)
    {
      ROS_INFO("Some error.");
    }
    
    
  }
  ROS_INFO("FPS:%f",(double)image_counter/etime);
//(std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) 
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
  if (SHOW_IMAGES)
  {
    cv::destroyWindow(WINDOW);
  }


  
  
}
