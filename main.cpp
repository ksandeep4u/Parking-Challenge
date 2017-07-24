#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types_conversion.h>

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
  
	// Load PLY file
  string fName = argv[1];

//	string fName = "/media/fabio/Sandeep_2TB/Parkopedia-Exercise/PointCloud_Inglostadt_Theater_10mm/pointcloud.ply"; //1.4GB
//	string fName = "/media/fabio/Sandeep_2TB/Parkopedia-Exercise/PointCloud_Inglostadt_Theater2_10mm/pointcloud.ply"; //1.3GB
//	string fName = "/media/fabio/Sandeep_2TB/Parkopedia-Exercise/PointCloud_5mm__wolfsburg_galerie_1b_01/pointcloud.ply"; //4GB
//	string fName = "/media/fabio/Sandeep_2TB/Parkopedia-Exercise/PointCloud_5mm__wolfsburg_galerie_1b_02/pointcloud.ply"; //3GB

	cout << "Loading point cloud: " << fName << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	auto flag1 = pcl::io::loadPLYFile(fName,*input_cloud);
	if(flag1 < 0)
	{
		cerr << "Error loading point cloud " << fName << std::endl;
		return -1;
	}
	cout << "Loaded " << input_cloud->width * input_cloud->height << " points." << endl;


	//generate a sub-cloud (floor) for quick processing
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
	float th = 0.1; //floor thickness

/*
	// select sub-cloud around z=0 plane (using a filter)
	cout << "Finding Z = 0 ... " << endl;
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(input_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-th,th);
	pass.filter(*floor_rgb);
	cout << "Finding Z = 0 ... Done" << endl;
*/

	// select sub-cloud around z=0 plane (manually)
	cout << "Finding Z = 0 (manually)... " << endl;
	for(unsigned int i = 0; i<input_cloud->points.size(); ++i)
	{
		if ((input_cloud->points[i].z > -th) && (input_cloud->points[i].z < th))
		{
			floor_rgb->push_back(input_cloud->points[i]);
		}
	}
	cout << "Finding Z = 0 (manually)... Done" << endl;

/*	
	//apply a color threshold on floor to detect parking lines
	cout << "Applying a color threshold (manually)... " << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_bw(new pcl::PointCloud<pcl::PointXYZRGB>()); //black and white cloud
	int th_color = 120; //threshold to detect white parking lines
	for (unsigned int i = 0; i<floor_rgb->points.size(); ++i)
	{
	  pcl::PointXYZRGB pt;
		pt.x = floor_rgb->points[i].x;
		pt.y = floor_rgb->points[i].y;
		pt.z = floor_rgb->points[i].z;
		if ((((int)floor_rgb->points[i].r)>th_color)&&(((int)floor_rgb->points[i].g)>th_color)&&(((int)floor_rgb->points[i].b)>th_color))
		{
			pt.r = 255;
			pt.g = 255;
			pt.b = 255;
		}
		else
		{
			pt.r = 0;
			pt.g = 0;
			pt.b = 0;
		}
		floor_bw->push_back(pt);
	}
*/

/*	
	//convert RGB to HSV
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr floor_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());;
	pcl::PointCloudXYZRGBtoXYZHSV(*floor_rgb,*floor_hsv);
	for (uint32_t i = 0; i<floor_hsv->points.size(); ++i)
	{
	  pcl::PointXYZRGB pt;
		pt.x = floor_rgb->points[i].x;
		pt.y = floor_rgb->points[i].y;
		pt.z = floor_rgb->points[i].z;
		if ((floor_hsv->points[i].s<0.4)&&(floor_hsv->points[i].v>0.6))
		{
			pt.r = 255;
			pt.g = 255;
			pt.b = 255;
		}
		else
		{
			pt.r = 0;
			pt.g = 0;
			pt.b = 0;
		}
		floor_bw->push_back(pt);
	}
	cout << "Applying a color threshold (manually)... Done" << endl;
*/



	// color-based segmentation
	cout << "Color-based segmentation ... " << endl;
	vector<pcl::PointIndices> clusters;
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;// = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>> (new pcl::search::kdTree<pcl::PointXYZRGB>);
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(floor_rgb);
	reg.setSearchMethod(tree);
	reg.setDistanceThreshold(10);
	reg.setPointColorThreshold(3);
	reg.setRegionColorThreshold(2);
	reg.setMinClusterSize(5000);
//	reg.setMaxClusterSize(120000);
	reg.extract(clusters);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusters_cloud = reg.getColoredCloud();
	cout << "Color-based segmentation ... Done" << endl;

	// Visualization
	cout << "Visualizing ... " << endl;
	pcl::visualization::PCLVisualizer viewer("Parking floor");
	int v1(0);
	viewer.createViewPort(0.0,0.0,0.5,1.0,v1);
	viewer.setBackgroundColor(0.5,0.5,0.5,v1); // Set background to a dark grey
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> f_rgb(floor_rgb); // Define R,G,B colors for the point cloud
	viewer.addPointCloud(floor_rgb,f_rgb,"floor",v1);

  // convert from XYZRGB to XYZ
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*floor_rgb,*tmp_cloud);

	// compute and draw bounding boxes of the cluster clouds
	for (uint32_t i = 0; i < clusters.size(); ++i)
	{
		pcl::PointXYZ min;
		pcl::PointXYZ max;
		pcl::PointXYZ pos;
		Eigen::Matrix3f rot_mat;
	
		pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
		pcl::PointIndices::Ptr cl_ptr = boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(clusters[i]));
		feature_extractor.setInputCloud(tmp_cloud);
		feature_extractor.setIndices(cl_ptr);
		feature_extractor.compute();
		bool flag = feature_extractor.getOBB(min,max,pos,rot_mat);
		if (flag)
		{
		  //compute box
			double width = max.x-min.x;
			double height = max.y-min.y;
			double depth = max.z-min.z;
			Eigen::Vector3f trans(pos.x,pos.y,pos.z);
			Eigen::Quaternionf quat(rot_mat);
			// draw box
			viewer.addCube(trans,quat,width,height,depth,"Box "+std::to_string(i));
		}
	}


	int v2(0);
	viewer.createViewPort(0.5,0.0,1.0,1.0,v2);
	viewer.setBackgroundColor(0.7,0.7,0.7,v2); // Set background to a dark grey
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> c_rgb(clusters_cloud); // Define R,G,B colors for the point cloud
	viewer.addPointCloud(clusters_cloud,c_rgb,"clusters",v2);

	while(!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	cout << "Visualizing ... Done" << endl;
  
  return 0;
}
