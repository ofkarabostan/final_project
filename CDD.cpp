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
  
  sn << "/home/hcaslan/Desktop/Closed_door_detection/PCDs_"<<pcdnum<<"/transformed_cloud_scene_" << samplenumber << ".pcd";
}

void CDD::readPCD()
{
  deque<double> dictionary;
    cout << "Loading file..." << endl;
    ifstream myfile ("angle.csv");
    if ( myfile.is_open() ) {
        copy(istream_iterator<double>(myfile),
             istream_iterator<double>(),
             back_inserter<deque<double>>(dictionary));
        myfile.close();
    } else {
        cout << "Unable to open file." << endl;
    }
  double gazeboAngle=dictionary[samplenumber-1];
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
  sstc << "/home/hcaslan/Desktop/Closed_door_detection/fromTransform/transform_"<< samplenumber << ".pcd";
  writer.write<pcl::PointXYZ> (sstc.str(), *cloud_transformed, false);
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
    reg.setMinClusterSize (900);
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
    viewer->addPointCloud(cloud_colored, "cloud");
    pcl::PCDWriter writer3;
    cn << "/home/hcaslan/Desktop/Closed_door_detection/fromRegionGrowing/colored_"<< samplenumber << ".pcd";
    writer3.write<pcl::PointXYZRGB> (cn.str(), *cloud_colored, false);  ///colored_cloud.pcd
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
    std::cout << "PointCloud representing the Cluster  "<< j+1 <<": " << cloud_cluster->size () << " data points." << std::endl;
    

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
        CDD::boundingBox(cloud_cluster);
        x.clear();
        
        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
        }
      std::stringstream rgsn;
      rgsn << "/home/hcaslan/Desktop/Closed_door_detection/fromExtraction/cluster_" << samplenumber<<'_'<< j+1 << ".pcd";
      writer2.write<pcl::PointXYZ> (rgsn.str(), *cloud_cluster, false);

/*
      cout<<"x="<<coefficients->values[0]<<endl;
      cout<<"y="<<coefficients->values[1]<<endl;
      cout<<"z="<<coefficients->values[2]<<endl;
      cout<<"0="<<coefficients->values[3]<<endl;

      coordinateVector.push_back(coefficients->values[0]);
      coordinateVector.push_back(coefficients->values[1]);
      coordinateVector.push_back(coefficients->values[2]);
      coordinateVector.push_back(coefficients->values[3]);
      coefficientsVector.push_back(coordinateVector);
      coordinateVector.clear();
*/
      
      
    j++;
  }
}

void CDD::boundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_cloud )
{
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> gt_FE;
  pcl::PointXYZ gt_min_point_AABB;
  pcl::PointXYZ gt_max_point_AABB;
  gt_FE.setInputCloud (dummy_cloud);
  gt_FE.setIndices(indices);
  gt_FE.compute();
  gt_FE.getAABB (gt_min_point_AABB, gt_max_point_AABB);

  std::cout<<"min_x  "<<gt_min_point_AABB.x<<std::endl;
  std::cout<<"max_x  "<<gt_max_point_AABB.x<<std::endl;
  std::cout<<"min_y  "<<gt_min_point_AABB.y<<std::endl;
  std::cout<<"max_y  "<<gt_max_point_AABB.y<<std::endl;
  std::cout<<"min_z  "<<gt_min_point_AABB.z<<std::endl;
  std::cout<<"max_z  "<<gt_max_point_AABB.z<<std::endl;
 
  x.push_back(gt_min_point_AABB.x);
  x.push_back(gt_max_point_AABB.x);
  x.push_back(gt_min_point_AABB.y);
  x.push_back(gt_max_point_AABB.y);
  x.push_back(gt_min_point_AABB.z);
  x.push_back(gt_max_point_AABB.z);
  clusterVector.push_back(x);
}

void CDD::boundingBoxPrint()
{
  for(int j=0; j<clusterVector.size(); j++){
    for(int i=0; i<6; i++){
      //std::cout<<clusterVector[j][i]<<std::endl;
      ssv <<"cube"<<j;
      viewer->addCube (clusterVector[i][0],clusterVector[i][1],clusterVector[i][2],
      clusterVector[i][3],clusterVector[i][4],clusterVector[i][5], 1.0, 1.0, 0.0,ssv.str());
      viewer->addCoordinateSystem();
    }
  }
}

void CDD::decision(void)
{
  vector<int> temp;
  for(int i=0; i<clusterVector.size(); i++){
    if(abs(double(clusterVector[i][5]-clusterVector[i][4]))<0.01){
        cout<<"cluster "<<i+1<<" is ground"<<endl;
        ground=i;
    }
    if((abs(double(clusterVector[i][3]-clusterVector[i][2])<0.05)) || (abs(double(clusterVector[i][1]-clusterVector[i][0])>0.02))) {
      if((abs(double(clusterVector[i][1]-clusterVector[i][0])) < 0.2)){
        cout<<"cluster "<<i+1<<" is part of frame"<<endl;
        frames.push_back(i);
      }
    }
    if((abs(double(clusterVector[i][3]-clusterVector[i][2]))>0.2) && abs(double((clusterVector[i][5]-clusterVector[i][4])>0.50)) && abs(double((clusterVector[i][1]-clusterVector[i][0])<0.0075))){
      //cout<<"cluster "<<i+1<<" is part of wall"<<endl;
      walls.push_back(i);
      if(walls.size()>2){
        if((abs(double(clusterVector[walls[0]][1]-clusterVector[walls[1]][1]))<0.01)){
          temp.push_back(walls[0]);
          temp.push_back(walls[1]);
          walls.clear();
          walls=temp;
          temp.clear();
        }
        else if((abs(double(clusterVector[walls[0]][1]-clusterVector[walls[2]][1]))<0.01)){
          temp.push_back(walls[0]);
          temp.push_back(walls[2]);
          walls.clear();
          walls=temp;
          temp.clear();
        }
        else if((abs(double(clusterVector[walls[1]][1]-clusterVector[walls[2]][1]))<0.01)){
          temp.push_back(walls[1]);
          temp.push_back(walls[2]);
          walls.clear();
          walls=temp;
          temp.clear();
        }
        else 
          cout<<"Failed"<<endl;
      }
    }
  }
  cout<<"cluster "<<walls[0]+1<<" is part of wall"<<endl;
  cout<<"cluster "<<walls[1]+1<<" is part of wall"<<endl;
}

void CDD::writerPCD(void)
{
  pcl::PCDWriter writer;
  
  
  writer.write<pcl::PointXYZ> (sstc.str(), *cloud_transformed, false);
}
///Destructor
CDD::~CDD()
{
}