//Gene Class
#include <iostream>
#include <string>
#include <stdlib.h>
class Gene
{
	private:
	int id;
	string Name;
	double Value;
	double Value_Max;
	double Value_Min;
	double Mutate_Scale;
	
	public:
	Gene(int,string,double);
	GetValue();
	Mutate();
	
};
Gene::Gene(int myid,string myName,double myValue)
{
	id = myid;
	Name = myName;
	Value = myValue;
}
double Gene::GetValue()
{
	return Value;
}
void Gene::Mutate()
{
	double temp = Value;
	
}
