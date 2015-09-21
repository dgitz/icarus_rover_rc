#ifndef __MAPPINGNODE_INCLUDED__   
#define __MAPPINGNODE_INCLUDED__
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
struct grid_cell{
	int index;
	int x;  //Grid coordinates
	int y;  //Grid coordinates
	double X;  //Real-world coordinates, Meters
	double Y;  //Real-world coordinates, Meters
	int status;  //1 if it is updated, 0 if not.
     int border;  //1 if cell is on border of map, 0 otherwise
     int value;
	std::vector<int> connected_list;
};
struct grid_point{
     int x;
     int y;
     double X;
     double Y;
     int index;
};
int get_index_from_cell(int x, int y);
grid_cell find_cell(grid_cell mycell, grid_cell map_origin,double X, double Y, int value);
grid_point find_grid_coord(grid_point mypoint,grid_cell map_origin);
//bool initialize_map(nav_msgs::OccupancyGrid& my_map,nav_msgs::MapMetaData& my_map_metadata);
bool initialize_map();
bool update_map();
#endif

//nav_msgs::OccupancyGrid

//void ICARUS_Sonar_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
