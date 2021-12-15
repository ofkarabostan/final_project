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
// destructor
Planes::~Planes()
{
}

double Planes::operator-( Planes&  x){
      double nom = abs(x.getA()*(-(*this).getD()/(*this).getA())+x.getD());
      double denom = sqrt(pow(x.getA(),2)+pow(x.getB(),2)+pow(x.getC(),2));
      return nom/denom; 
    }

int Planes::plane_number=1;

///Constructor
CDD::CDD(boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
:viewer(v)
{
	cloud_raw=pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_normals=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
	cloud_colored=pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_filtered=pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_transformed=pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
}

void CDD::removingNans()
{
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud(*cloud_raw, *indices);
}

void CDD::getSample()
{
  int pcdnum;
  std::cout<< "Enter the Condition" << std::endl;
  std::cout<< "************************************" << std::endl;
  std::cout<< "1---Orange   (   1 - 130 )" << std::endl;
  std::cout<< "2---Red      ( 131 - 396 )" << std::endl;
  std::cout<< "3---Green    ( 397 - 957 )" << std::endl;
  std::cout<< "4---Yellow   ( 958 - 1223)" << std::endl;
  std::cout<< "5---Blue     (1224 - 1353)" << std::endl;
  std::cout<< "************************************" << std::endl;
  std::cin >> pcdnum;
  std::cout<< "Enter the Sample Number" << std::endl;
  std::cin >> samplenumber;
  key<<"/home/omer/Desktop/CDD/";
  sn << key.str() <<"PCDs_"<<pcdnum<<"/transformed_cloud_scene_" << samplenumber << ".pcd";
}

void CDD::readPCD()
{
  deque<double> angles;
    cout << "Loading file..." << endl;
    ifstream C ("angle.csv");
    if ( C.is_open() ) {
        copy(istream_iterator<double>(C),
             istream_iterator<double>(),
             back_inserter<deque<double>>(angles));
        C.close();
    } else {
        cout << "Unable to open file." << endl;
    }
  double gazeboAngle=angles[samplenumber-1];
  cout <<"Angle for "<<samplenumber<<"= "<<gazeboAngle<< endl;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(sn.str(),*cloud_raw) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
	}
  
  //transform the point cloud in order to change its coordinate system    

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate (Eigen::AngleAxisf (gazeboAngle+M_PI/2, Eigen::Vector3f::UnitZ()) );
  pcl::transformPointCloud (*cloud_raw, *cloud_transformed, transform);
  pcl::PCDWriter writer;
  std::stringstream tempss;
  tempss <<key.str()<< "fromTransform/transform_"<< samplenumber << ".pcd";
  writer.write<pcl::PointXYZ> (tempss.str(), *cloud_transformed, false);
  tempss.clear();
}

void CDD::showPointCloud()
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rawcloudColor(cloud_raw, 0, 255, 0);
	viewer->addPointCloud(cloud_raw, rawcloudColor, "cloud raw");
}

void CDD::regionGrowing(void)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud_transformed);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (1500);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud_transformed);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    reg.extract (cluster_indices);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_colored= reg.getColoredCloud (); //to create "colored_cloud"
    //viewer->addPointCloud(cloud_colored, "cloud");
    pcl::PCDWriter writer3;
    std::stringstream tempss;
    tempss << key.str()<<"fromRegionGrowing/colored_"<< samplenumber << ".pcd";
    writer3.write<pcl::PointXYZRGB> (tempss.str(), *cloud_colored, false);  ///colored_cloud.pcd
    tempss.clear();
}

void CDD::clusterExtraction(void)
{
  int j = 0;
  pcl::PCDWriter writer2;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : it->indices)
      cloud_cluster->push_back ((*cloud_transformed)[idx]);
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;  
    //std::cout << "PointCloud representing the Cluster  "<< j+1 <<": " << cloud_cluster->size () << " data points." << std::endl;
    

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud_cluster);
    seg.segment (*inliers, *coefficients);

    if(coefficients->values[0]>0.95){
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud_cluster);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0, 2);
      pass.filter (*cloud_cluster);
    }

    pcl::PointXYZ min_points;
    pcl::PointXYZ max_points;
    CDD::boundingBox(cloud_cluster,min_points,max_points);
    p.push_back(Planes(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3],
    min_points.x,max_points.x,min_points.y,max_points.y,min_points.z,max_points.z));


    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given clusterVector.\n");
    }
    std::stringstream rgsn;
    rgsn << key.str()<<"fromExtraction/cluster_" << samplenumber<<'_'<< j+1 << ".pcd";
    writer2.write<pcl::PointXYZ> (rgsn.str(), *cloud_cluster, false);
  
    j++;
  }
}

//+
void CDD::boundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_cloud ,pcl::PointXYZ & min_points,pcl::PointXYZ & max_points)
{
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> gt_FE;
  gt_FE.setInputCloud (dummy_cloud);
  gt_FE.setIndices(indices);
  gt_FE.compute();
  gt_FE.getAABB (min_points, max_points);
}
//+
void CDD::boundingBoxPrint()
{
  std::stringstream tempss;
  for(int i=0; i<p.size(); i++){
    tempss <<"cube"<<i;
    viewer->addCube (p[i].getMinX(),p[i].getMaxX(),p[i].getMinY(),p[i].getMaxY(),p[i].getMinZ(),p[i].getMaxZ(), 1.0, 0.0, 0.0,tempss.str());
    }
  viewer->addCoordinateSystem();
  tempss.clear();
}

void CDD::decision(void)
{
  vector<Planes> wall;

  for(int i=0;i<p.size();i++){
    if(p[i].getA()>0.98 )
    {
      wall.push_back(p[i]);
    }
    else if(p[i].getC()>0.98 && abs(p[i].getMaxY()-p[i].getMinY()>0.18))
    {
      cout<<p[i].getName()<<" is the ground"<<endl;
    }
    //to find the frames,the difference between y_min and y_max values of the clusters must be lower than 0.05  
    //or the difference between x_min and x_max values of the clusters must be greater then 0.02
    else if( abs(p[i].getMinY()-p[i].getMaxY())<0.05 ||
     (abs(p[i].getMinX()-p[i].getMaxX())>0.02 && abs(p[i].getMinX()-p[i].getMaxX())<0.2))
    {
      cout<<p[i].getName() <<" is a frame"<<endl;
    }
  }
  

if(wall.size()<3)
{
  cout<<"There is no door!"<<endl;
}


else if ((abs(wall[1].getMaxX()-wall[2].getMinX()<0.02) || abs(wall[1].getMinX()-wall[2].getMaxX()<0.02)) &&
   ( (abs(wall[1].getMaxY()-wall[2].getMinY())<1.0 && abs(wall[1].getMaxY()-wall[2].getMinY())>0.8)||
     (abs(wall[2].getMaxY()-wall[1].getMinY())<1.0 && abs(wall[2].getMaxY()-wall[1].getMinY())>0.8)) &&
     (abs(wall[0].getMaxX()-wall[1].getMinX())>0.09|| abs(wall[0]-wall[2])>0.09 )
)
{
  cout<<wall[1].getName()<<" is a wall"<<endl;
  cout<<wall[2].getName()<<" is a wall"<<endl;
  cout<<wall[0].getName()<<" is the door"<<endl;
}
else if ((abs(wall[0].getMaxX()-wall[1].getMinX()<0.02) || abs(wall[0].getMinX()-wall[1].getMaxX()<0.02)) &&
   ( (abs(wall[0].getMaxY()-wall[1].getMinY())<1.0 && abs(wall[0].getMaxY()-wall[1].getMinY())>0.8)||
     (abs(wall[1].getMaxY()-wall[0].getMinY())<1.0 && abs(wall[1].getMaxY()-wall[0].getMinY())>0.8)) &&
     (abs(wall[2].getMaxX()-wall[0].getMinX())>0.09|| abs(wall[0]-wall[2])>0.09 )
)
{
  cout<<wall[0].getName()<<" is a wall"<<endl;
  cout<<wall[1].getName()<<" is a wall"<<endl;
  cout<<wall[2].getName()<<" is the door"<<endl;
}
else if ((abs(wall[0].getMaxX()-wall[2].getMinX()<0.02) || abs(wall[0].getMinX()-wall[2].getMaxX()<0.02)) &&
   ( (abs(wall[0].getMaxY()-wall[2].getMinY())<1.0 && abs(wall[0].getMaxY()-wall[2].getMinY())>0.8)||
     (abs(wall[2].getMaxY()-wall[0].getMinY())<1.0 && abs(wall[2].getMaxY()-wall[0].getMinY())>0.8)) &&
     (abs(wall[0].getMaxX()-wall[1].getMinX())>0.09|| abs(wall[1]-wall[2])>0.09 )
)
{
  cout<<wall[0].getName()<<" is a wall"<<endl;
  cout<<wall[2].getName()<<" is a wall"<<endl;
  cout<<wall[1].getName()<<" is the door"<<endl;
}


}

//I couldn't find the reason why it doesn't work. instead writer is defined and called after each operation.
void CDD::writerPCD(void)
{
  std::stringstream tempss;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (tempss.str(),*cloud_temp, false);
  tempss.clear();
}
//for experimental purpose :)
void CDD::whatHappened(void)
{
  ofstream OK;
  string mystr;
  std::stringstream ofss;
  ofss <<key.str()<< "OK"<<".csv";
  OK.open(ofss.str(),std::fstream::app);
  OK<<samplenumber<<',';
  cout<<"OK?"<<endl;
  cin>>mystr;
  OK<<mystr;
  OK<<'\n';
  OK.close();
}

//inside clusterMinMax -> 
//outfile_samplenumber.csv ->
//in row order      min_x max_x min_y max_y min_z max_z
//in colomb order   cluster 1 cluster 2 cluster 2 ....
void CDD::clustersMinMax(void)
{
/*  ofstream outfile;
  std::stringstream ofss;
  ofss <<key.str()<<"clusterMinMax/"<< "outfile_"<<samplenumber<<".csv";
  outfile.open(ofss.str());
  for (auto& row : clusterVector) {
  for (auto col : row)
    outfile << col <<',';
  outfile << '\n';
  }
  outfile << '\n';
  outfile<<"=B1-A1"<<','<<"=D1-C1"<<','<<"=F1-E1";
  outfile.close();*/
}
///Destructor
CDD::~CDD()
{
}