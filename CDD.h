
#ifndef CDD_H
#define CDD_H

///C++
#include <iostream>
#include <vector>
#include <string>
#include <bits/stdc++.h>
#include <math.h>
///PCL
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/pcl_config.h> //for PCL Version check
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>
#include <fstream>

///
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


/*//PCL Libraries
#include <pcl/filters/passthrough.h> //passthrough
#include <pcl/filters/voxel_grid.h> // voxel grid
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_key.h>
*/

/*///Create Folder
#include <sys/stat.h>
#include <sys/types.h>
*/

using namespace std;
class CDD
{
public:
    CDD(boost::shared_ptr<pcl::visualization::PCLVisualizer>);
    ~CDD();
    ///
    int samplenumber;
    ///stringstreams
    std::stringstream sn;
    std::stringstream cn;
    std::stringstream ssv;
    std::stringstream sstc;
    int ground;
    vector<int>frames;
    int door;
    vector<int>walls;
    ///vectors
    std::vector<int> inliers;
    vector<double>x;
    vector<vector<double>> clusterVector;
    vector<vector<double>> coefficientsVector;
    vector<double>coordinateVector;
    std::vector <pcl::PointIndices> cluster_indices;
    ///functions
    void readPCD();
    void getSample();
    void showPointCloud();
    void showClusterCloud();
    void removingNans();
    void RANSAC();
    void regionGrowing();
    void clusterExtraction();
    void writerPCD();
    void boundingBoxPrint();
    void decision();
    void boundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr );
    
private:
    ///clouds
    pcl::PointCloud<pcl::PointXYZ>		::Ptr 			cloud_raw;
    pcl::PointCloud<pcl::Normal>		::Ptr 			cloud_normals;
    pcl::PointCloud<pcl::PointXYZRGB>		::Ptr 			cloud_colored;
    pcl::PointCloud<pcl::PointXYZ>		::Ptr 			cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>		::Ptr 			cloud_transformed;
    ///
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer;
    pcl::IndicesPtr indices;
    
};
#endif
