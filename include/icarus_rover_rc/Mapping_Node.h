#ifndef __MAPPINGNODE_INCLUDED__   
#define __MAPPINGNODE_INCLUDED__
struct grid_cell{
	int index;
	int x;  //Grid coordinates
	int y;  //Grid coordinates
	double X;  //Real-world coordinates, Meters
	double Y;  //Real-world coordinates, Meters
	int status;  //1 if it is updated, 0 if not.
};

int get_index_from_cell(int x, int y);
grid_cell update_cell(grid_cell mycell, grid_cell map_origin,double X, double Y, int value);
#endif