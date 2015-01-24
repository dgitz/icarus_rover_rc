#include <ros/ros.h>
#include "../include/icarus_rover_rc/Evolution.h"
/*void say_hello()
{
	ROS_INFO_STREAM("Hello,World!");
}
*/
Gene::Gene(int myid)
{
  createGene(myid);
}

void Gene::createGene(int myid)
{
  id = myid;
}  
int Gene::getGene()
{
  return id;
}
