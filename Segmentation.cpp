#include "Segmentation.h"

Segmentation::Segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr)
{
	input_ptr = cloud_ptr;
}

void Segmentation::segment(double dist_th, double color_th1, double color_th2)
{
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;// = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>> (new pcl::search::kdTree<pcl::PointXYZRGB>);
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(input_ptr);
	reg.setSearchMethod(tree);
	reg.setDistanceThreshold(dist_th);
	reg.setPointColorThreshold(color_th1);
	reg.setRegionColorThreshold(color_th2);
//	reg.setMinClusterSize(5000);
//	reg.setMaxClusterSize(120000);
	reg.extract(clusters);
	clusters_ptr = reg.getColoredCloud();
	
	//ToDo: new ideas to improve the clusters
}  

void Segmentation::computeBBoxes(double area_upper, double area_lower)
{
  // convert from XYZRGB to XYZ
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*input_ptr,*tmp_ptr);

	// compute and draw bounding boxes of the clusters on the floor
	for (uint32_t i = 0; i < clusters.size(); ++i)
	{
    // extract features
		pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
		pcl::PointXYZ min;
		pcl::PointXYZ max;
		pcl::PointXYZ pos;
		Eigen::Matrix3f rot_mat;
		pcl::PointIndices::Ptr cl_ptr = boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(clusters[i]));
		feature_extractor.setInputCloud(tmp_ptr);
		feature_extractor.setIndices(cl_ptr);
		feature_extractor.compute();
		bool flag_fe = feature_extractor.getOBB(min,max,pos,rot_mat);
		if (flag_fe)
		{
		  //compute box
			bbox bb;
			bb.width = max.x-min.x;
			bb.height = max.y-min.y;
			bb.depth = max.z-min.z;
			bb.trans = Eigen::Vector3f(pos.x,pos.y,pos.z);
			bb.quat = Eigen::Quaternionf(rot_mat);

      //refinement: remove too small or too big clusters			
			if (((bb.width*bb.height) < area_upper) && ((bb.width*bb.height) > area_lower))
			{
			  boundingBoxes.push_back(bb);
		  }
		}
	}
}

std::vector<bbox>& Segmentation::getBBoxes()
{
  return boundingBoxes;
}

void Segmentation::visualize()
{
  //cluster cloud
  pcl::visualization::PCLVisualizer viewer0("Parking clusters");
	int v0(0);
	viewer0.setBackgroundColor(0.7,0.7,0.7,v0); // Set background to a dark grey
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> c_rgb(clusters_ptr); // Define R,G,B colors for the point cloud
	viewer0.addPointCloud(clusters_ptr,c_rgb,"clusters",v0);

	while(!viewer0.wasStopped())
	{
		viewer0.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	//result with boxes
	pcl::visualization::PCLVisualizer viewer1("Parking result");
	int v1(0);
	viewer1.setBackgroundColor(0.5,0.5,0.5,v1); // Set background to a dark grey
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> i_rgb(input_ptr); // Define R,G,B colors for the point cloud
	viewer1.addPointCloud(input_ptr,i_rgb,"input",v1);
	
	// draw bounding boxes of the clusters on the floor
	for (uint32_t i = 0; i < boundingBoxes.size(); ++i)
	{
	  viewer1.addCube(boundingBoxes[i].trans,boundingBoxes[i].quat,boundingBoxes[i].width,boundingBoxes[i].height,boundingBoxes[i].depth,"Box "+std::to_string(i));
	}
	
	while(!viewer1.wasStopped())
	{
		viewer1.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

Segmentation::~Segmentation()
{


}

