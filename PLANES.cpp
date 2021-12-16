///C++
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
///PCL
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
///headers
#include "CDD.h"

using namespace std;


// contructor
Planes::Planes(double a, double b, double c, double d,double min_x,double max_x,double min_y,double max_y,double min_z,double max_z) 
: a(a), b(b), c(c), d(d),min_x(min_x),max_x(max_x),min_y(min_y),max_y(max_y),min_z(min_z),max_z(max_z)
{
  
  
  std::stringstream sn;
  sn << "plane" << plane_number;
  name = sn.str();
  this->printPlane();
  plane_number++;

}

// getter methods
//plane coeffcients getter methods
double Planes::getA(){ return this->a;}
double Planes::getB(){ return this->b;}
double Planes::getC(){ return this->c;}
double Planes::getD(){ return this->d;}
//bounding box min-max getter methods
double Planes::getMinX(){ return this->min_x;}
double Planes::getMaxX(){ return this->max_x;}
double Planes::getMinY(){ return this->min_y;}
double Planes::getMaxY(){ return this->max_y;}
double Planes::getMinZ(){ return this->min_z;}
double Planes::getMaxZ(){ return this->max_z;}
//plane name getter method
string Planes::getName(){ return this->name;}

// setter methods
//plane coeffcients setter methods
void Planes::setA(double a){ this->a=a;}
void Planes::setB(double b){ this->b=b;}
void Planes::setC(double c){ this->c=c;}
void Planes::setD(double d){ this->d=d;}
//bounding box min-max setter methods
void Planes::setMinX(double min_x){ this->min_x=min_x;}
void Planes::setMaxX(double max_x){ this->max_x=max_x;}
void Planes::setMinY(double min_y){ this->min_y=min_y;}
void Planes::setMaxY(double max_y){ this->max_y=max_y;}
void Planes::setMinZ(double min_z){ this->min_z=min_z;}
void Planes::setMaxZ(double max_z){ this->max_z=max_z;}// print data 
void Planes::printPlane(){
  std:: cout<< plane_number <<". plane is "<<"("<<setw(12)<< this->a<<")" << "x " 
                                               <<"("<<setw(12)<< this->b <<")"<< "y " 
                                               <<"("<<setw(12)<< this->c <<")"<< "z " 
                                               <<"("<<setw(12)<< this->d <<")"<< " = 0 " 
                                              <<" min_x= "<<setw(12)<<min_x<<" max_x= "<<setw(12)<<max_x
                                              <<" min_y= "<<setw(12)<<min_y<<" max_y= "<<setw(12)<<max_y
                                              <<" min_z= "<<setw(12)<<min_z<<" max_z= "<<setw(12)<<max_z<<endl;
}

double Planes::operator-( Planes&  x){
      double nom = abs(x.getA()*(-(*this).getD()/(*this).getA())+x.getD());
      double denom = sqrt(pow(x.getA(),2)+pow(x.getB(),2)+pow(x.getC(),2));
      return nom/denom; 
    }

int Planes::plane_number=1;

// destructor
Planes::~Planes()
{
}
