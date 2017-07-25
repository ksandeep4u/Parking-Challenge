#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types_conversion.h>

#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/moment_of_inertia_estimation.h>

typedef struct bbox
{
	double width;
	double height;
	double depth;
	Eigen::Vector3f trans;
	Eigen::Quaternionf quat;
} bbox;

class Segmentation
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_ptr;   //input point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusters_ptr;//clusters point cloud
  std::vector<pcl::PointIndices> clusters;            //cluster indices
  std::vector<bbox> boundingBoxes;                    //bounding box of each cluster
  
public:
  Segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
  
  void segment(double dist_th, double color_th1, double color_th2); // parameters: thresholds used in segmentation
  
  void computeBBoxes(double area_upper, double area_lower); //parameters: area_upper, area_lower represent the upper and the lower limits of the size of a parking bay

  std::vector<bbox>& getBBoxes(); //returns the computed bounding boxes
  
  void visualize(); // visualizes the clusters and the bounding boxes

  ~Segmentation();
  
};

#endif /* SEGMENTATION_H_ */
