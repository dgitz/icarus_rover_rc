#ifndef __EVOLUTION_INCLUDED__   
#define __EVOLUTION_INCLUDED__

//void say_hello();
class Gene
{
private:
  int id;
  Gene(){};
public:
  Gene(int myid);
  void createGene(int myid);
  int getGene();
};

#endif

