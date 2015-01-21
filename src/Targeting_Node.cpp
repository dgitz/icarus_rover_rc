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
#include "icarus_rover_rc/Definitions.h"
#include "icarus_rover_rc/ICARUS_Target_Status.h"

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image w/ Overlay";
static const char WINDOW1[] = "Helper Image 1";
int SHOW_IMAGES = 0;
string detect_method;
string filter_method;
string image_folder;
string image_filename;
string image_filepath;
string capture_date;
string picked_date;
string target_filename;
fstream target_file;
string result_filename;
fstream result_file;
string root_directory;
string template_image_name;
string template_image_path;
int erode_value;
int dilate_value;
int threshold_value;
int run_live;
int image_ready = 0;
int Target_Type = TARGET_NONE;

//target_file.open(
int targets[2][2];
int target_counter = 0;
image_transport::Publisher pub;
cv_bridge::CvImagePtr cv_ptr;
  cv::Mat sensor_image;
cv::Mat orig_sensor_image;
cv::Mat template_image;
cv::Mat raw_image;

double evaluate_target(double target_x, double target_y,double target1_x, double target1_y,double target2_x,double target2_y,double height,double width)
{
  double score;
  if ((target_x < 0) and (target_y < 0) and (target1_x < 0)) //No Target Found and No Target Present
  {
    score = 0.0;
  }
  else if((target_x < 0) and (target1_x > 0))  //No Target Found and Target Present
  {
    score = double(height*width);
  }
  else if((target_x > 0) and (target1_x < 0)) //Target Found and No Target Present
  {
    score = double(height*width);
  }
  else
  {
    double score1,score2;
    score1 = sqrt(pow((target_x-target1_x),2)+pow((target_y-target1_y),2));
    score2 = sqrt(pow((target_x-target2_x),2)+pow((target_y-target2_y),2));
    score = min(score1,score2);
  }
  return score;
}
cv::Mat process_image(cv::Mat image,int threshold,int erode,int dilate)
{
  cv::Mat processed_image;
  cv::blur(image,processed_image,Size(5,5));
  //cv::Sobel(processed_image,processed_image,CV_8U,1,0,3,1,0);
  //cv::threshold(processed_image,processed_image,0,255,CV_THRESH_OTSU+CV_THRESH_BINARY);
   if (SHOW_IMAGES)
	  {
	    
      imshow(WINDOW1,processed_image);
	    cv::waitKey(1);
	  }
  return processed_image;
}
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
        orig_sensor_image = cv_ptr->image.clone();
        cvtColor(orig_sensor_image, orig_sensor_image, CV_BGR2GRAY);
        cv::resize(orig_sensor_image, orig_sensor_image, Size(640, 480), 0, 0, INTER_CUBIC);
        image_ready = 1;
        
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
	
	

        //pub.publish(cv_ptr->toImageMsg());
}
  

int main(int argc, char **argv)
{ 


  

  ros::init(argc, argv, "targeting");
  ros::NodeHandle nh;
  //nh.getParam("target_count",target_count);
  nh.getParam("root_directory",root_directory);
  nh.getParam("capture_date",capture_date);
  nh.getParam("picked_date",picked_date);
  nh.getParam("template_image_name",template_image_name);
  nh.getParam("show_images",SHOW_IMAGES);
  nh.getParam("detect_method",detect_method);
  nh.getParam("filter_method",filter_method);
  nh.getParam("threshold",threshold_value);
  nh.getParam("run_live",run_live);
  nh.getParam("erode",erode_value);
  nh.getParam("dilate",dilate_value);
  ros::Publisher Pub_ICARUS_Target_Status = nh.advertise<icarus_rover_rc::ICARUS_Target_Status>("ICARUS_Target_Status", 1000);  
  ::icarus_rover_rc::ICARUS_Target_Status Target_Status;
  if (SHOW_IMAGES)
  {
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW1,CV_WINDOW_AUTOSIZE);
  }
  double fps = 0.0;
	template_image_path = root_directory + "media/Templates/" + template_image_name + ".png";
	raw_image = imread(template_image_path.c_str(),CV_LOAD_IMAGE_GRAYSCALE);
	cv::resize(raw_image,template_image, Size(), 3, 3, INTER_LINEAR );
	template_image = process_image(template_image,threshold_value,erode_value,dilate_value);
	ros::Rate loop_rate(1000);
	image_folder = root_directory + "media/imagesamples-" + capture_date + "/";
	target_filename = root_directory + "target_files/" + picked_date + ".csv";
	target_file.open(target_filename.c_str(), fstream::in);
	vector<string> image_names = vector<string>();
	string tempstr;
	int image_count = image_names.size()-2;
	vector<KeyPoint> template_keypoints,sensor_keypoints;
	cv::Mat template_descriptor,sensor_descriptor;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
	//SURF Initialization Stuff.  
	int minHessian = 1000;
	cv::FeatureDetector * surf_detector = new cv::SURF(minHessian);
	cv::DescriptorExtractor * surf_extractor = new cv::SURF(minHessian);
	cv::FlannBasedMatcher surf_matcher;
  
	//ORB Initialization Stuff
	int numKeyPoints = 1500;
	float distThreshold = 15.0;
	cv::FeatureDetector * orb_detector = new cv::ORB();
	cv::DescriptorExtractor * orb_extractor = new cv::ORB();
	if (detect_method.compare("SURF") == 0)
	{
	  surf_detector->detect(template_image,template_keypoints);
	  surf_extractor->compute(template_image,template_keypoints,template_descriptor);
	  
	}
	if (detect_method.compare("ORB") == 0)
	{
	  orb_detector->detect(template_image,template_keypoints);
	  orb_extractor->compute(template_image,template_keypoints,template_descriptor);
	}
 
	vector<DMatch>matches;
	std::clock_t    start;
	double lasttime = 0.0;
  double dtime = 0.0;
	int image_counter = 0;
	double total_score = 0.0;
	double myscore = 0.0;
	int Height = 1;
	int Width = 1;
  int target1_x;
  int target1_y;
  int target2_x;
  int target2_y;
  double target_x,target_y = 0.0;
  string token,remain;
	while( ros::ok() && true)
	{
    
	  lasttime = std::clock();
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  if (run_live == 1)
    {
      
    }
    else
    {
      
      image_counter++;
      getline(target_file,tempstr);
  	  
	    token = tempstr.substr(0,tempstr.find(","));
	    remain = tempstr.substr(token.length()+1);
	    
	    image_filename = token;
	    
	    token = remain.substr(0,remain.find(","));
	    remain = remain.substr(token.length()+1);
	    target1_x = std::atoi(token.c_str());
	    
	    token = remain.substr(0,remain.find(","));
	    remain = remain.substr(token.length()+1);
	    target1_y = std::atoi(token.c_str());    
	    
	    token = remain.substr(0,remain.find(","));
	    remain = remain.substr(token.length()+1);
	    target2_x = std::atoi(token.c_str()); 

	    token = remain.substr(0,remain.find(","));
	    target2_y = std::atoi(token.c_str());     
	    image_ready = 1;
	    //ROS_INFO("T1_x:%d T1_y:%d T2_x:%d T2_y:%d",target1_x,target1_y,target2_x,target2_y);
	    image_filepath = image_folder + image_filename;
	    orig_sensor_image = imread(image_filepath.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    }
    if (image_ready == 0)
    {
      
      continue;
    }
    target_x = 0.0;
    target_y = 0.0;
    sensor_image = orig_sensor_image.clone();
	  sensor_image = process_image(sensor_image,threshold_value,erode_value,dilate_value);
	  cv::Size s = sensor_image.size();
	  Height = s.height;
	  Width = s.width;
	  try
	  {
	    
	    if (detect_method.compare("SURF") == 0)
	    {
	      surf_detector->detect(sensor_image,sensor_keypoints);
	      surf_extractor->compute(sensor_image,sensor_keypoints,sensor_descriptor);
	      surf_matcher.match(template_descriptor,sensor_descriptor,matches);
	     
	    }
	    if (detect_method.compare("ORB") == 0)
	    {
	      orb_detector->detect(sensor_image,sensor_keypoints);
	      orb_extractor->compute(sensor_image,sensor_keypoints,sensor_descriptor);
        cv::BFMatcher orb_matcher(cv::NORM_HAMMING2);
        orb_matcher.match(template_descriptor,sensor_descriptor,matches);  
	    }
	  
	  
	    double max_dist = 0;
	    double min_dist = 100;
      double dist = 0.0;
	    for( int i = 0; i < template_descriptor.rows; i++ )
	    { 
	      dist = matches[i].distance;
	      if( dist < min_dist ) min_dist = dist;
	      if( dist > max_dist ) max_dist = dist;
	    }
	    std::vector< DMatch > good_matches;
	    //surf_matcher.knnMatch(template_descriptor, des_image, matches, 2);
	    for( int i = 0; i < template_descriptor.rows; i++ )
	    { 
	      if( matches[i].distance < 3*min_dist )
	      { 
		      good_matches.push_back( matches[i]); 
	      }
	    }

	    //-- Draw only "good" matches
	    Mat img_matches;
	    //if (detect_method.compare("SURF") == 0)
	    //{
	      drawMatches( template_image, template_keypoints, sensor_image, sensor_keypoints,
		         good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		         vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	    //}

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

	  //-- Show detected matches
    double min_x,max_x,min_y,max_y = 0.0;
    string tempstr = "";
	  for(int i = 0;i<4;i++)
	  {
	      target_x += scene_corners[i].x;
	      target_y += scene_corners[i].y;
        
    }
	  target_x = target_x/4;
	  target_y = target_y/4;
	  if ((good_matches.size() < 4) or (target_x < 0) or (target_y < 0))
	  {
	    target_x = -1.0;
	    target_y = -1.0;
	  }

	  if (SHOW_IMAGES)
	  {
	    //cv::circle(img_matches,Point(target_x,target_y),5,cv::Scalar(0,0,255),CV_FILLED,8,0);
	    //imshow( "Good Matches & Object detection", img_matches );
      cv::circle(orig_sensor_image,Point(target_x,target_y),15,cv::Scalar(0,0,255),CV_FILLED,8,0);
      cv::resize(orig_sensor_image,orig_sensor_image, Size(), .5, .5, INTER_LINEAR );
      std::ostringstream ss;
      ss << "FPS:" << 1/dtime;
      tempstr = ss.str();
      cv::putText(orig_sensor_image, tempstr.c_str(), cvPoint(30,30), 
    FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,255), 1, CV_AA);
      imshow(WINDOW,orig_sensor_image);
	    cv::waitKey(1);
	  }
    dtime = (std::clock() - lasttime) / (double)(CLOCKS_PER_SEC /1);
       
	    
	  }
	  catch(const std::exception& ex)
	  {
	    ROS_INFO("ERROR:%s",ex.what());
	    target_x = -1.0;
	    target_y = -1.0;
	  }
    if (run_live == 0)
    {
	    myscore = evaluate_target(target_x,target_y,target1_x,target1_y,target2_x,target2_y,Height,Width);
	    total_score += myscore;
    }
	  //ROS_INFO("Processed Image:%s with Score:%f",image_filename.c_str(),myscore);
   	if (image_ready == 1)
    {
      image_ready = 0;
    }
    Target_Status.header.stamp = ros::Time::now();
    Target_Status.header.frame_id = "Target_Status";
    Target_Status.Target_Type = Target_Type;
    //Target_Status.Target_Point.x = target_x;
    //Target_Status.Target_Point.y = target_y;
    Pub_ICARUS_Target_Status.publish(Target_Status);    
	  
    ROS_INFO("FPS: %f x: %f y: %f",1.0/dtime,target_x,target_y);
  }
  if (run_live==0)
  {
	  target_file.close();
	  total_score = 100*(1.0-total_score/(image_counter*Height*Width));
  }
	if (SHOW_IMAGES)
	{
	  cv::destroyWindow(WINDOW);
    cv::destroyWindow(WINDOW1);
	}


  
  
}

/*

*/
/*
cv::Mat results;
		  cv::Mat dists;
		  std::vector<std::vector<cv::DMatch> > matches;
		  int k=2; // find the 2 nearest neighbors
		  bool useBFMatcher = true; // SET TO TRUE TO USE BRUTE FORCE MATCHER
		  if(template_descriptor.type()==CV_8U)
		  {
			  // Binary descriptors detected (from ORB, Brief, BRISK, FREAK)
			  ROS_INFO("Binary descriptors detected...\n");
			  if(useBFMatcher)
			  {
				  cv::BFMatcher matcher(cv::NORM_HAMMING); // use cv::NORM_HAMMING2 for ORB descriptor with WTA_K == 3 or 4 (see ORB constructor)
				  matcher.knnMatch(template_descriptor, sensor_descriptor, matches, k);
			  }
			  else
			  {
				  // Create Flann LSH index
				  cv::flann::Index flannIndex(sensor_descriptor, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);

				  // search (nearest neighbor)
				  flannIndex.knnSearch(template_descriptor, results, dists, k, cv::flann::SearchParams() );
			  }
         ROS_INFO("HERE1\n");
		  }
		  else
		  {
			  // assume it is CV_32F
			  printf("Float descriptors detected...\n");
			  if(useBFMatcher)
			  {
				  cv::BFMatcher matcher(cv::NORM_L2);
				  matcher.knnMatch(template_descriptor, sensor_descriptor, matches, k);
			  }
			  else
			  {
				  // Create Flann KDTree index
				  cv::flann::Index flannIndex(sensor_descriptor, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);

				  // search (nearest neighbor)
				  flannIndex.knnSearch(template_descriptor, results, dists, k, cv::flann::SearchParams() );
			  }
		  }
      // Conversion to CV_32F if needed
		  if(dists.type() == CV_32S)
		  {
			  cv::Mat temp;
			  dists.convertTo(temp, CV_32F);
			  dists = temp;
		  }
      ROS_INFO("HERE2\n");
      // Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
		  float nndrRatio = 0.8f;
		  std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
		  std::vector<int> indexes_1, indexes_2; // Used for homography
		  std::vector<uchar> outlier_mask;  // Used for homography
		  // Check if this descriptor matches with those of the objects
		  if(!useBFMatcher)
		  {
			  for(int i=0; i<sensor_descriptor.rows; ++i)
			  {
          
				  // Apply NNDR
				  //printf("q=%d dist1=%f dist2=%f\n", i, dists.at<float>(i,0), dists.at<float>(i,1));
				  if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 &&
				     dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
				    {
              
					    mpts_1.push_back(template_keypoints.at(i).pt);
					    indexes_1.push_back(i);

					    mpts_2.push_back(sensor_keypoints.at(results.at<int>(i,0)).pt);
					    indexes_2.push_back(results.at<int>(i,0));
				    }
          ROS_INFO("HERE3B\n");
			  }
        
		  }
		  else
		  {
			  for(unsigned int i=0; i<matches.size(); ++i)
			  {
          
				  // Apply NNDR
				  //printf("q=%d dist1=%f dist2=%f\n", matches.at(i).at(0).queryIdx, matches.at(i).at(0).distance, matches.at(i).at(1).distance);
				  if(matches.at(i).size() == 2 &&
				     matches.at(i).at(0).distance <= nndrRatio * matches.at(i).at(1).distance)
			    {
				    mpts_1.push_back(template_keypoints.at(matches.at(i).at(0).queryIdx).pt);
				    indexes_1.push_back(matches.at(i).at(0).queryIdx);

				    mpts_2.push_back(sensor_keypoints.at(matches.at(i).at(0).trainIdx).pt);
				    indexes_2.push_back(matches.at(i).at(0).trainIdx);
			    }
          
			  }
		  }
      
      // FIND HOMOGRAPHY
		  unsigned int minInliers = 8;
		  if(mpts_1.size() >= minInliers)
		  {
			  //time.start();
        ROS_INFO("HERE4\n");
			  cv::Mat H = findHomography(mpts_1,
					  mpts_2,
					  cv::RANSAC,
					  1.0,
					  outlier_mask);
			      //Mat H = findHomography( obj, scene, CV_RANSAC );

          //-- Get the corners from the image_1 ( the object to be "detected" )
          std::vector<Point2f> obj_corners(4);
          obj_corners[0] = cvPoint(0,0); 
          obj_corners[1] = cvPoint( template_image.cols, 0 );
          obj_corners[2] = cvPoint( template_image.cols, template_image.rows ); 
          obj_corners[3] = cvPoint( 0, template_image.rows );
          std::vector<Point2f> scene_corners(4);

          perspectiveTransform( obj_corners, scene_corners, H);
          ROS_INFO("HERE5\n");
*/
