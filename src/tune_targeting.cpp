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
int threshold_min;
int threshold_max;
int threshold_count;
int erode_min;
int erode_max;
int dilate_min;
int dilate_max;


//target_file.open(
int targets[2][2];
int target_counter = 0;
image_transport::Publisher pub;
cv_bridge::CvImagePtr cv_ptr;
  cv::Mat sensor_image;
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
  cv::equalizeHist( image, image );
  cv::threshold(image, image, threshold, 255, CV_THRESH_BINARY);
  cv::Mat element_erode = getStructuringElement(cv::MORPH_CROSS,
              cv::Size(2 * erode + 1, 2 * erode + 1),
              cv::Point(erode, erode) );
  cv::Mat element_dilate = getStructuringElement(cv::MORPH_CROSS,
              cv::Size(2 * dilate + 1, 2 * dilate + 1),
              cv::Point(dilate, dilate) );
       // Apply erosion or dilation on the image
  cv::erode(image,image,element_erode); 
  cv::dilate(image,image,element_dilate);
  if(filter_method.compare("DIGITAL") == 0)
  {
  }
  return image;
}
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
  nh.getParam("detect_method",detect_method);
  nh.getParam("filter_method",filter_method);
  nh.getParam("threshold_min",threshold_min);
  nh.getParam("threshold_max",threshold_max);
  nh.getParam("threshold_count",threshold_count);
  nh.getParam("erode_max",erode_max);
  nh.getParam("dilate_max",dilate_max);

  int THRESHOLDs[threshold_count];
  THRESHOLDs[0] = threshold_min;
  for(int i=1;i<=(threshold_count+1);i++)
  {
    THRESHOLDs[i] = THRESHOLDs[i-1]+threshold_max/threshold_count;
  }
  int ERODEs[erode_max];
  for(int i=0;i<erode_max;i++)
  {
    ERODEs[i]=i+1;
  }
  int DILATEs[dilate_max];
  for(int i=0;i<dilate_max;i++)
  {
    DILATEs[i]=i+1;
  }
  int threshold_index = 0;
  int erode_index = 0;
  int dilate_index = 0;

  result_filename = root_directory + "result_files/" + capture_date + "_" + picked_date + ".csv";
  result_file.open(result_filename.c_str(), fstream::out);
  result_file << "Threshold" << "," << "Erode" << "," << "Dilate" << "," << "Score" << "," << "FPS" << endl;
        
  for(threshold_index=0;threshold_index<=(threshold_count+1);threshold_index++)
  {
    for(erode_index=0;erode_index<erode_max;erode_index++)
    {
      for(dilate_index=0;dilate_index<dilate_max;dilate_index++)
      {
        double fps = 0.0;
        template_image_path = root_directory + "media/Templates/" + template_image_name + ".png";
        raw_image = imread(template_image_path.c_str(),CV_LOAD_IMAGE_GRAYSCALE);
        cv::resize(raw_image,template_image, Size(), 3, 3, INTER_LINEAR );
        template_image = process_image(template_image,THRESHOLDs[threshold_index],ERODEs[erode_index],DILATEs[dilate_index]);
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
        double etime = 0.0;
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
          sensor_image = imread(image_filepath.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
          sensor_image = process_image(sensor_image,THRESHOLDs[threshold_index],ERODEs[erode_index],DILATEs[dilate_index]);
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

            //-- Draw only "good" matches
            Mat img_matches;
            if (detect_method.compare("SURF") == 0)
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
            
            cv::circle(img_matches,Point(target_x,target_y),5,cv::Scalar(0,0,255),CV_FILLED,8,0);
            imshow( "Good Matches & Object detection", img_matches );
            cv::waitKey(1);
          }
            etime += (std::clock() - start) / (double)(CLOCKS_PER_SEC /1);
            
          }
          catch(const std::exception& ex)
          {
            //ROS_INFO("ERROR:%s",ex.what());
            target_x = -1.0;
            target_y = -1.0;
          }
          myscore = evaluate_target(target_x,target_y,target1_x,target1_y,target2_x,target2_y,Height,Width);
          total_score += myscore;
          //ROS_INFO("Processed Image:%s with Score:%f",image_filename.c_str(),myscore);
        }
        target_file.close();
        total_score = 100*(1.0-total_score/(image_counter*Height*Width));
        fps = (double)image_counter/etime;
        if (etime < 1)
        {
          fps = 0.0;
        }
        ROS_INFO("Threshold:%d Erode:%d Dilate:%d Error Rate:%f",THRESHOLDs[threshold_index],ERODEs[erode_index],DILATEs[dilate_index],total_score);
        ROS_INFO("FPS:%f",fps);
        result_file << THRESHOLDs[threshold_index] << "," << ERODEs[erode_index] << "," << DILATEs[dilate_index] << "," << total_score <<  "," << fps << endl;
        if (SHOW_IMAGES)
        {
          cv::destroyWindow(WINDOW);
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
