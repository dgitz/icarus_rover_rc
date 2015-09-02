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
static const char WINDOW1[] = "Helper Image 1";
int SHOW_IMAGES = 0;
string detect_method;
string filter_method;
string image_folder;
string image_filename;
string image_filepath;
string capture_date;
string picked_date;
string result_name;
string target_filename;
fstream target_file;
string result_filename;
fstream result_file;
string root_directory;
string template_image_name;
string template_image_path;

int param1_start,param1_stop,param1_count;
int param2_start,param2_stop,param2_count;
int param3_start,param3_stop,param3_count;
int param4_start,param4_stop,param4_count;
int param5_start,param5_stop,param5_count;
string param1_name,param2_name,param3_name,param4_name,param5_name;


//target_file.open(
int targets[2][2];
int target_counter = 0;
image_transport::Publisher pub;
cv_bridge::CvImagePtr cv_ptr;
  cv::Mat sensor_image,orig_sensor_image;
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
    score = 800.0;//double(height*width);
  }
  else if((target_x > 0) and (target1_x < 0)) //Target Found and No Target Present
  {
    score = 800.0;//double(height*width);
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

cv::Mat process_image(cv::Mat image,int value1, int value2, int value3, int value4, int value5)
{
  cv::Mat processed_image;
  try
  {
    //processed_image = image;
    cv::blur(image,processed_image,Size(value1,value1));
    cv::equalizeHist( image, processed_image );
  //cv::Sobel(processed_image,processed_image,CV_8U,value2,value3,value4,1,0);
  //cv::threshold(processed_image,processed_image,0,255,CV_THRESH_OTSU+CV_THRESH_BINARY);
   if (SHOW_IMAGES)
	  {
	    
      imshow(WINDOW1,processed_image);
	    cv::waitKey(1);
	  }
  }
  catch(const std::exception& ex)
  {
    processed_image = cv::Mat::zeros( 640, 480, CV_8U );
  }
  return processed_image;
}
int main(int argc, char **argv)
{


  
  if (SHOW_IMAGES)
  {
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW1,CV_WINDOW_AUTOSIZE);
  }
  ros::init(argc, argv, "target_tuning");
  ros::NodeHandle nh;
  //nh.getParam("target_count",target_count);
  nh.getParam("root_directory",root_directory);
  nh.getParam("capture_date",capture_date);
  nh.getParam("picked_date",picked_date);
  nh.getParam("result_name",result_name);
  nh.getParam("template_image_name",template_image_name);
  nh.getParam("show_images",SHOW_IMAGES);
  nh.getParam("detect_method",detect_method);
  nh.getParam("filter_method",filter_method);
 
  nh.getParam("param1_start",param1_start);
  nh.getParam("param1_stop",param1_stop);
  nh.getParam("param1_count",param1_count);
  nh.getParam("param1_name",param1_name);

  nh.getParam("param2_start",param2_start);
  nh.getParam("param2_stop",param2_stop);
  nh.getParam("param2_count",param2_count);
  nh.getParam("param2_name",param2_name);

  nh.getParam("param3_start",param3_start);
  nh.getParam("param3_stop",param3_stop);
  nh.getParam("param3_count",param3_count);
  nh.getParam("param3_name",param3_name);

  nh.getParam("param4_start",param4_start);
  nh.getParam("param4_stop",param4_stop);
  nh.getParam("param4_count",param4_count);
  nh.getParam("param4_name",param4_name);

  nh.getParam("param5_start",param5_start);
  nh.getParam("param5_stop",param5_stop);
  nh.getParam("param5_count",param5_count);
  nh.getParam("param5_name",param5_name);

  int PARAM1s[param1_count];
  int PARAM2s[param2_count];
  int PARAM3s[param3_count];
  int PARAM4s[param4_count];
  int PARAM5s[param5_count];
  
  PARAM1s[0] = param1_start;
  PARAM2s[0] = param2_start;
  PARAM3s[0] = param3_start;
  PARAM4s[0] = param4_start;
  PARAM5s[0] = param5_start;
  if (param1_count > 0)
  {
    for(int i=1;i<param1_count;i++)
    {
      PARAM1s[i] = PARAM1s[i-1] + (int)((param1_stop-param1_start+1)/param1_count);
    }
  }
  else
  {
    for(int i=0;i<param1_count;i++)
    {
      PARAM1s[i] = 0;
    }
  }
  if (param2_count > 0)
  {
    for(int i=1;i<param2_count;i++)
    {
      PARAM2s[i] = PARAM2s[i-1] + (int)((param2_stop-param2_start+1)/param2_count);
    }
  }
  else
  {
     for(int i=0;i<param2_count;i++)
    {
      PARAM2s[i] = 0;
    }
  }
  if (param3_count > 0)
  {
    for(int i=1;i<param3_count;i++)
    {
      PARAM3s[i] = PARAM3s[i-1] + (int)((param3_stop-param3_start+1)/param3_count);
    }
  }
  else
  {
    for(int i=0;i<param3_count;i++)
    {
      PARAM3s[i] = 0;
    }
  }
  if (param4_count > 0)
  {
    for(int i=1;i<param4_count;i++)
    {
      PARAM4s[i] = PARAM4s[i-1] + (int)((param4_stop-param4_start+1)/param4_count);
    }
  }
  else
  {
    for(int i=0;i<param4_count;i++)
    {
      PARAM4s[i] = 0;
    }
  }
  if (param5_count > 0)
  {
    for(int i=0;i<param5_count;i++)
    {
      PARAM5s[i] = 0;
    }
  }
  else
  {
    for(int i=0;i<param5_count;i++)
    {
      PARAM5s[i] = 0;
    }
  }
  

  result_filename = root_directory + "result_files/" + result_name + ".csv";
  result_file.open(result_filename.c_str(), fstream::out);
  result_file << param1_name << "," << param2_name << "," << param3_name << "," << param4_name << "," << param5_name << "," << "Score,"  << "FPS" << endl;
        
  for(int param1_index=0;param1_index<param1_count;param1_index++)
  {
    if(param1_count < 1) { continue; }
    for(int param2_index=0;param2_index<param2_count;param2_index++)
    {
      if(param2_count < 1) { continue; }
      for(int param3_index=0;param3_index<param3_count;param3_index++)
      {
        if(param3_count < 1) { continue; }
        for(int param4_index=0;param4_index<param4_count;param4_index++)
        {
          if(param4_count < 1) { continue; }
          for(int param5_index=0;param5_index<param5_count;param5_index++)
          {
            if(param5_count < 1) { continue; }
            
            ROS_INFO("%s:%d %s:%d %s:%d %s:%d %s:%d",param1_name.c_str(),PARAM1s[param1_index],param2_name.c_str(),PARAM2s[param2_index],param3_name.c_str(),PARAM3s[param3_index],
              param4_name.c_str(),PARAM4s[param4_index],param5_name.c_str(),PARAM5s[param5_index]);
            double fps = 0.0;
            template_image_path = root_directory + "media/Templates/" + template_image_name + ".png";
            raw_image = imread(template_image_path.c_str(),CV_LOAD_IMAGE_GRAYSCALE);
            cv::resize(raw_image,template_image, Size(), 3, 3, INTER_LINEAR );
            template_image = process_image(template_image,PARAM1s[param1_index],PARAM2s[param2_index],PARAM3s[param3_index],PARAM4s[param4_index],PARAM5s[param5_index]);
            ros::Rate loop_rate(1000);
            image_folder = root_directory + "media/imagesamples-" + capture_date + "/";
            target_filename = root_directory + "target_files/" + picked_date + ".csv";
            target_file.open(target_filename.c_str(), fstream::in);
            vector<string> image_names = vector<string>();
            string tempstr;
            int image_count = image_names.size()-2;
            vector<KeyPoint> template_keypoints,sensor_keypoints;
            cv::Mat template_descriptor,sensor_descriptor;

            int numKeyPoints = 1500;
	          float distThreshold = 15.0;
	          cv::FeatureDetector * orb_detector = new cv::ORB();
	          cv::DescriptorExtractor * orb_extractor = new cv::ORB();
	          if (detect_method.compare("ORB") == 0)
	          {
	            orb_detector->detect(template_image,template_keypoints);
	            orb_extractor->compute(template_image,template_keypoints,template_descriptor);
	          }
           
	          vector<DMatch>matches;
	            
                    
            std::clock_t    start;
            
            double etime,dtime = 0.0;
            int image_counter = 0;
            double total_score = 0.0;
            double myscore = 0.0;
            int Height = 1;
            int Width = 1;
            while(getline(target_file,tempstr) && ros::ok() && true)
            {
              
              ros::spinOnce();
              loop_rate.sleep();
              image_counter++;
              start = std::clock();
              string token,remain;
              token = tempstr.substr(0,tempstr.find(","));
              remain = tempstr.substr(token.length()+1);
              
              image_filename = token;
              
              token = remain.substr(0,remain.find(","));
              remain = remain.substr(token.length()+1);
              int target1_x = std::atoi(token.c_str());
              
              token = remain.substr(0,remain.find(","));
              remain = remain.substr(token.length()+1);
              int target1_y = std::atoi(token.c_str());    
              
              token = remain.substr(0,remain.find(","));
              remain = remain.substr(token.length()+1);
              int target2_x = std::atoi(token.c_str()); 

              token = remain.substr(0,remain.find(","));
              int target2_y = std::atoi(token.c_str());     
              double target_x = 0.0; 
              double target_y = 0.0;
              
              //ROS_INFO("T1_x:%d T1_y:%d T2_x:%d T2_y:%d",target1_x,target1_y,target2_x,target2_y);
              image_filepath = image_folder + image_filename;
              orig_sensor_image = imread(image_filepath.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
              sensor_image = orig_sensor_image.clone();
              sensor_image = process_image(sensor_image,PARAM1s[param1_index],PARAM2s[param2_index],PARAM3s[param3_index],PARAM4s[param4_index],PARAM5s[param5_index]);
              //cv::Size s = sensor_image.size();
              Height = 480;
              Width = 640;
              try
              {
                
                if (detect_method.compare("ORB") == 0)
	              {
	                orb_detector->detect(sensor_image,sensor_keypoints);
	                orb_extractor->compute(sensor_image,sensor_keypoints,sensor_descriptor);
                  cv::BFMatcher orb_matcher(cv::NORM_HAMMING2);
                  orb_matcher.match(template_descriptor,sensor_descriptor,matches);  
	              }
              
                double max_dist = 0;
                double min_dist = 100;
                for( int i = 0; i < template_descriptor.rows; i++ )
                { 
                  double dist = matches[i].distance;
                  if( dist < min_dist ) min_dist = dist;
                  if( dist > max_dist ) max_dist = dist;
                }
                
                //ROS_INFO("-- Max dist : %f \n", max_dist );
                //ROS_INFO("-- Min dist : %f \n", min_dist );

                std::vector< DMatch > good_matches;
                //surf_matcher.knnMatch(template_descriptor, des_image, matches, 2);
                for( int i = 0; i < template_descriptor.rows; i++ )
                { 
                  if( matches[i].distance < 3*min_dist )
                  { 
                    good_matches.push_back( matches[i]); 
                  }
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

              //-- Show detected matches


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
                  cv::circle(orig_sensor_image,Point(target_x,target_y),15,cv::Scalar(0,0,255),CV_FILLED,8,0);
                  cv::resize(orig_sensor_image,orig_sensor_image, Size(), .5, .5, INTER_LINEAR );
                  std::ostringstream ss;
                  ss << "FPS:" << 1/dtime;
                  tempstr = ss.str();
                  cv::putText(orig_sensor_image, tempstr.c_str(), cvPoint(30,30),FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,255), 1, CV_AA);
                  imshow(WINDOW,orig_sensor_image);
	                cv::waitKey(1);
              
                }
                dtime = (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
                etime += dtime;
                
              }
              catch(const std::exception& ex)
              {
                //ROS_INFO("ERROR:%s",ex.what());
                target_x = -1.0;
                target_y = -1.0;
              }
              myscore = evaluate_target(target_x,target_y,target1_x,target1_y,target2_x,target2_y,Height,Width);
              total_score += myscore;
              ROS_INFO("Processed Image:%s with Score:%f",image_filename.c_str(),myscore);
            }
            target_file.close();
            total_score = 100*(1.0-total_score/(image_counter*800));
            fps = (double)image_counter/etime;
            if (etime < 1)
            {
              fps = 0.0;
            }
            ROS_INFO("%s:%d %s:%d %s:%d %s:%d %s:%d Success Rate:%f FPS:%f",param1_name.c_str(),PARAM1s[param1_index],param2_name.c_str(),PARAM2s[param2_index],param3_name.c_str(),PARAM3s[param3_index],
              param4_name.c_str(),PARAM4s[param4_index],param5_name.c_str(),PARAM5s[param5_index],total_score,fps);
            dtime = 0.0;
            etime = 0.0;
            result_file << PARAM1s[param1_index] << "," << PARAM2s[param2_index] << "," << PARAM3s[param3_index] << "," << PARAM4s[param4_index] << "," << PARAM5s[param5_index] << "," << total_score <<  "," << fps << endl;
            if (SHOW_IMAGES)
            {
              cv::destroyWindow(WINDOW);
              cv::destroyWindow(WINDOW1);
            }
          }
        }
      }
    }
  }
  result_file.close();


  
  
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
