#ifndef PLANES_H
#define PLANES_H

#include "CDD.h"
using namespace std;

class Planes
{

  private:
    double min_x,max_x,min_y,max_y,min_z,max_z;
    double a,b,c,d;
    string name;
  public:

    static int plane_number ;
    // getter methods
    //plane coeffcients getter methods
    double getA();
    double getB();
    double getC();
    double getD();
    //bounding box min-max getter methods
    double getMinX();
    double getMaxX();
    double getMinY();
    double getMaxY();
    double getMinZ();
    double getMaxZ();

    //plane name getter method
    string getName();

    // setter methods
    //plane coeffcients setter methods
    void setA(double);
    void setB(double);
    void setC(double);
    void setD(double);
    //bounding box min-max setter methods
    void setMinX(double);
    void setMaxX(double);
    void setMinY(double);
    void setMaxY(double);
    void setMinZ(double);
    void setMaxZ(double);
    // print data 
    void printPlane();
    // contructor
    Planes(double, double, double, double,double, double, double, double,double, double );
    //destructor
    ~Planes();
    double operator-( Planes&  x);
   
};

#endif