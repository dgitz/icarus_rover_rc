#ifndef __EVOLUTION_INCLUDED__   
#define __EVOLUTION_INCLUDED__

//void say_hello();
class Gene
{
private:
  double value;
  double min_value;
  double max_value;
  Gene(){};
public:
  Gene(double myvalue, double mymin_value, double mymax_value);
  void initGene(double myvalue, double mymin_value, double mymax_value);
  double getValue();
  void mutateGene();
};

class Population
{
public:
  Gene Recombine(Gene gene1, Gene gene2);
};
#endif

