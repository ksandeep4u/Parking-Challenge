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

//#include <pcl/filters/passthrough.h>

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
	// find the floor
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


/*
  //-----------------------------------------------------------------------------
	//apply a color threshold on floor to detect parking lines
  //-----------------------------------------------------------------------------

	pcl::PointCloud<pcl::PointXYZI>::Ptr floor_bw_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	cout << "Applying a color threshold ... " << endl;
	//convert RGB to HSV
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr floor_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());;
	pcl::PointCloudXYZRGBtoXYZHSV(*floor_cloud,*floor_hsv);
	double sat_th = 0.4; //threshold on saturation
	double val_th = 0.6; //threshold on intensity value
	for (uint32_t i = 0; i<floor_hsv->points.size(); ++i)
	{
	  pcl::PointXYZI pt;
		pt.x = floor_hsv->points[i].x;
		pt.y = floor_hsv->points[i].y;
		pt.z = floor_hsv->points[i].z;
		if ((floor_hsv->points[i].s < sat_th) && (floor_hsv->points[i].v > val_th))
		{
			pt.intensity = 255;
		}
		else
		{
			pt.intensity = 0;
		}
		floor_bw_cloud->push_back(pt);
	}
	cout << "Applying a color threshold ... Done" << endl;
*/


  //-----------------------------------------------------------------------------
	// generate a sub-cloud for a quick testing
  //-----------------------------------------------------------------------------

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	//find the centroid of the floor cloud
	pcl::PointXYZ cp;	//centroid
	auto flag_centroid = pcl::computeCentroid(*floor_cloud,cp);
	// select sub-cloud around origin using kdtree
	cout << "Finding kNN ... " << endl;
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
	cout << "Finding kNN ... Done" << endl;


  //-----------------------------------------------------------------------------
	// color-based segmentation of the sub-cloud
  //-----------------------------------------------------------------------------

	cout << "Color-based segmentation ... " << endl;
	vector<pcl::PointIndices> clusters;
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;// = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>> (new pcl::search::kdTree<pcl::PointXYZRGB>);
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(sub_cloud);
	reg.setSearchMethod(tree);
	reg.setDistanceThreshold(10);
	reg.setPointColorThreshold(3);
	reg.setRegionColorThreshold(2);
//	reg.setMinClusterSize(5000);
//	reg.setMaxClusterSize(120000);
	reg.extract(clusters);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusters_cloud = reg.getColoredCloud();
	cout << "Color-based segmentation ... Done" << endl;

  //-----------------------------------------------------------------------------
	// Visualization
  //-----------------------------------------------------------------------------

	cout << "Visualizing ... " << endl;
	
  //cluster cloud
  pcl::visualization::PCLVisualizer viewer0("Parking clusters");
	int v0(0);
//	viewer.createViewPort(0.5,0.0,1.0,1.0,v0); // right view
	viewer0.setBackgroundColor(0.7,0.7,0.7,v0); // Set background to a dark grey
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> c_rgb(clusters_cloud); // Define R,G,B colors for the point cloud
	viewer0.addPointCloud(clusters_cloud,c_rgb,"clusters",v0);

	while(!viewer0.wasStopped())
	{
		viewer0.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	//result with boxes
	pcl::visualization::PCLVisualizer viewer1("Parking result");
	int v1(0);
//	viewer.createViewPort(0.0,0.0,0.5,1.0,v1); //left view
	viewer1.setBackgroundColor(0.5,0.5,0.5,v1); // Set background to a dark grey
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> s_rgb(sub_cloud); // Define R,G,B colors for the point cloud
	viewer1.addPointCloud(sub_cloud,s_rgb,"sub-floor",v1);
	
  // convert from XYZRGB to XYZ
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*sub_cloud,*tmp_cloud);

	// compute and draw bounding boxes of the clusters on the floor
	for (uint32_t i = 0; i < clusters.size(); ++i)
	{
	  cout << i << "/" << clusters.size() << endl;
		pcl::PointXYZ min;
		pcl::PointXYZ max;
		pcl::PointXYZ pos;
		Eigen::Matrix3f rot_mat;
    // extract features
		pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
		pcl::PointIndices::Ptr cl_ptr = boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(clusters[i]));
		feature_extractor.setInputCloud(tmp_cloud);
		feature_extractor.setIndices(cl_ptr);
		feature_extractor.compute();
		bool flag_fe = feature_extractor.getOBB(min,max,pos,rot_mat);
		if (flag_fe)
		{
		  //compute box
			double width = max.x-min.x;
			double height = max.y-min.y;
			double depth = max.z-min.z;
			Eigen::Vector3f trans(pos.x,pos.y,pos.z);
			Eigen::Quaternionf quat(rot_mat);
			if (((width*height) < (3*6)) && ((width*height) > (2*2))) //refine clusters
			{
			  // draw box
			  cout << width << "," << height << "," << depth << endl;
			  viewer1.addCube(trans,quat,width,height,depth,"Box "+std::to_string(i));
		  }
		}
	}

	while(!viewer1.wasStopped())
	{
		viewer1.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	
	cout << "Visualizing ... Done" << endl;
  
  return 0;
}
