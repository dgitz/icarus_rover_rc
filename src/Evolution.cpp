#include <ros/ros.h>
#include "../include/icarus_rover_rc/Evolution.h"
Gene::Gene(double myvalue=0.0, double mymin_value=0.0, double mymax_value=0.0)
{
  value = myvalue;
  min_value = mymin_value;
  max_value = mymax_value;
  //initGene(myvalue,mymin_value,mymax_value);
}

/*void Gene::initGene(double myvalue, double mymin_value, double mymax_value)
{
  value = myvalue;
  min_value = mymin_value;
  max_value = mymax_value;
} */ 
void Gene::mutateGene()
{
}
double Gene::getValue()
{
  return value;
}
/*
Gene Population::Recombine(Gene gene1,Gene gene2)
{
  return gene1;
}
*/
