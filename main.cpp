/*
  Author: Sandeep Katragadda
  Email: s.katragadda@qmul.ac.uk
*/

#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
/*
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types_conversion.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/moment_of_inertia_estimation.h>
*/

//#include <pcl/filters/passthrough.h>

#include "Segmentation.h"
#include "Color_Threshold.h"

using namespace std;

int main(int argc, char* argv[])
{
  cout << "Welcome to the parking project" << endl;
  
  if (argc !=2 )
  {
		cerr << "Incorrect number of arguments" << endl;
    cerr << "Usage: " << argv[0] << "<fileName.ply>" << endl;
    return -1;
  }
  
  //-----------------------------------------------------------------------------
	// Load PLY file
  //-----------------------------------------------------------------------------
  string fName = argv[1];

//	string fName = "/media/fabio/Sandeep_2TB/Parkopedia-Exercise/PointCloud_Inglostadt_Theater_10mm/pointcloud.ply"; //1.4GB
//	string fName = "/media/fabio/Sandeep_2TB/Parkopedia-Exercise/PointCloud_Inglostadt_Theater2_10mm/pointcloud.ply"; //1.3GB
//	string fName = "/media/fabio/Sandeep_2TB/Parkopedia-Exercise/PointCloud_5mm__wolfsburg_galerie_1b_01/pointcloud.ply"; //4GB
//	string fName = "/media/fabio/Sandeep_2TB/Parkopedia-Exercise/PointCloud_5mm__wolfsburg_galerie_1b_02/pointcloud.ply"; //3GB

	cout << "Loading point cloud: " << fName << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	auto flag_load = pcl::io::loadPLYFile(fName,*input_cloud);
	if(flag_load < 0)
	{
		cerr << "Error loading point cloud " << fName << std::endl;
		return -1;
	}
	cout << "Loaded " << input_cloud->width * input_cloud->height << " points." << endl;


  //-----------------------------------------------------------------------------
	// Preprocessing: find the floor
  //-----------------------------------------------------------------------------
  
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	double thickness = 0.1; //floor thickness

/*
	// select sub-cloud around z=0 plane (using a filter)
	cout << "Finding Z = 0 ... " << endl;
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(input_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-thickness/2.0,thickness/2.0);
	pass.filter(*floor_cloud);
	cout << "Finding Z = 0 ... Done" << endl;
*/

	// select sub-cloud around z=0 plane (own logic)
	cout << "Finding floor (Z = 0) ... " << endl;
	for(uint32_t i = 0; i<input_cloud->points.size(); ++i)
	{
		if ((input_cloud->points[i].z > -thickness/2.0) && (input_cloud->points[i].z < thickness/2.0))
		{
			floor_cloud->push_back(input_cloud->points[i]);
		}
	}
	cout << "Finding floor (Z = 0) ... Done" << endl;


  //-----------------------------------------------------------------------------
	// Feature 1: extract white parking lines by applying a color threshold
  //-----------------------------------------------------------------------------
	cout << "Applying a color threshold ... " << endl;
	Color_Threshold *ct = new Color_Threshold(floor_cloud);
	double sat_th = 0.4; //threshold on saturation
	double val_th = 0.6; //threshold on intensity value
	ct->apply(sat_th,val_th);
	cout << "Applying a color threshold ... Done" << endl;
  
	// Visualization
	cout << "Visualizing white portions ... " << endl;
	ct->visualize();
	
/*	
  // Visualize from main()
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_ptr = ct->getOutput();
	pcl::visualization::PCLVisualizer viewer2("Parking lines");
	int v2(0);
	viewer2.setBackgroundColor(0.5,0.5,0.5,v2); // Set background to a dark grey
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> r_rgb(result_ptr); // Define R,G,B colors for the point cloud
	viewer2.addPointCloud(result_ptr,r_rgb,"lines",v2);
    	
	while(!viewer2.wasStopped())
	{
		viewer2.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
*/	
	cout << "Visualizing white portions ... Done" << endl;



  //-----------------------------------------------------------------------------
	// generate a sub-cloud for a quick testing
  //-----------------------------------------------------------------------------

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	//find the centroid of the floor cloud
	pcl::PointXYZ cp;	//centroid
	auto flag_centroid = pcl::computeCentroid(*floor_cloud,cp);
	// select sub-cloud around origin using kdtree
	cout << "Finding kNN (12m from floor centroid) ... " << endl;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*floor_cloud,*in_cloud);
	kdtree.setInputCloud(in_cloud);
	int rad = 12; //12meters
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquareDistance;
	auto flag = kdtree.radiusSearch(cp,rad,pointIdxRadiusSearch,pointRadiusSquareDistance);
	if(flag > 0)
	{
		for(size_t i=0; i<pointIdxRadiusSearch.size(); ++i)
		{
			sub_cloud->push_back(floor_cloud->points[pointIdxRadiusSearch[i]]);
		}
	}
//	cout << pointIdxRadiusSearch.size() << " points are selected for quick processing." << endl;
	cout << "Finding kNN (12m from floor centroid) ... Done" << endl;


  //-----------------------------------------------------------------------------
	// Feature 2: color-based segmentation of the sub-cloud
  //-----------------------------------------------------------------------------
	cout << "Color-based segmentation ... " << endl;
	Segmentation *seg = new Segmentation(sub_cloud);
	seg->segment(10,3,2);
	seg->computeBBoxes(3*6,2*2);
	cout << "Color-based segmentation ... Done" << endl;
	
	// Visualization
	cout << "Visualizing segments ... " << endl;
	seg->visualize();
	std::vector<bbox>& output_boxes = seg->getBBoxes();
	
	// Print output 
	for (uint32_t i = 0; i < output_boxes.size(); ++i)
	{
	  cout << "\n Box " << i << " of " << output_boxes.size() << endl;
	  cout << "Trans: " << output_boxes[i].trans << endl;
	  //cout << "Trans: " << output_boxes[i].trans[0] << "," << output_boxes[i].trans[1] << "," << output_boxes[i].trans[2] << endl;
	  cout << "Quat: " << output_boxes[i].quat.w() << "; " << output_boxes[i].quat.vec() << endl;
	  cout << "Dimensions: " << output_boxes[i].width << "," << output_boxes[i].height << "," << output_boxes[i].depth << endl;
	}
	cout << "Visualizing segments ... Done" << endl;
 

  //ToDo:
  //-----------------------------------------------------------------------------
	// Feature 3: extract other features of the sub-cloud (e.g. 3D edges, matched templates)
  //-----------------------------------------------------------------------------
 



  //ToDo:
  //-----------------------------------------------------------------------------
	// Feature fusion: exploit all the extracted features (white portions, color segments etc.) to find parking bays
	//-----------------------------------------------------------------------------
  
  
  
  return 0;
}
