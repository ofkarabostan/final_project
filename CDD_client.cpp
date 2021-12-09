#include <iostream>
#include "CDD.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> v(new pcl::visualization::PCLVisualizer("viewer"));



int main(int argc, char** argv)
{
    CDD C1(v);
    C1.getSample();
    C1.readPCD();
    //C1.showPointCloud();
    C1.regionGrowing();
    C1.clusterExtraction();
    //C1.writerPCD();
    C1.boundingBoxPrint();
    C1.decision();
    while (!v->wasStopped())
	{
		v->spinOnce();
        
	}

    return 0;
}