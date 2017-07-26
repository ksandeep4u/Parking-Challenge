#ifndef COLOR_THRESHOLD_H_
#define COLOR_THRESHOLD_H_

#include <iostream>
#include <string>

//#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types_conversion.h>
#include <pcl/impl/point_types.hpp>

class Color_Threshold
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_ptr;   //input point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_ptr;//clusters point cloud
  
public:
  Color_Threshold(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
  
  void apply(double sat_th, double val_th); // parameters: thresholds on saturation and intensity

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& getOutput();
  
  void visualize(); // visualizes the clusters and the bounding boxes

  ~Color_Threshold();
  
};

#endif /* SEGMENTATION_H_ */
