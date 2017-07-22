#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main()
{
  cout << "Welcome to the parking project" << endl;

	// Load PLY file
	string fName = "../../PointCloud_Inglostadt_Theater2_10mm/pointcloud.ply";
	cout << "Loading point cloud: " << fName << endl;

//	pcl::PCLPointCloud2 input_cloud;
//	pcl::io::loadPLYFile(fName, input_cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	auto flag1 = pcl::io::loadPLYFile(fName,*input_cloud);
	if(flag1 < 0)
	{
		cerr << "Error loading point cloud " << fName << std::endl;
		return -1;
	}
	cout << "Loaded " << input_cloud->width * input_cloud->height << " points." << endl;


	// Visualization
	cout << "Visualizing ... " << endl;
	pcl::visualization::PCLVisualizer viewer(" PCL Visualizer");
	viewer.setBackgroundColor(0.01,0.01,0.01,0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> i_rgb(input_cloud);
	viewer.addPointCloud(input_cloud,i_rgb,"input",0);

	//add coordinate system
	viewer.addCoordinateSystem(1.0);

	//add a sphere at origin
	pcl::PointXYZ op;
	op.x = 0;
	op.y = 0;
	op.z = 0;
	viewer.addSphere(op,1,0.5,0.5,0.0,"origin");

	//add axes
	pcl::PointXYZ xp;
	xp.x = 20;
	xp.y = 0;
	xp.z = 0;
	viewer.addLine<pcl::PointXYZ> (op, xp, 1.0, 0.0, 0.0, "x-axis");

	pcl::PointXYZ yp;
	yp.x = 0;
	yp.y = 20;
	yp.z = 0;
	viewer.addLine<pcl::PointXYZ> (op, yp, 0.0, 1.0, 0.0, "y-axis");

	pcl::PointXYZ zp;
	zp.x = 0;
	zp.y = 0;
	zp.z = 20;
	viewer.addLine<pcl::PointXYZ> (op, zp, 0.0, 0.0, 1.0, "z-axis");

	while(!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}

	cout << "Visualizing ... Done" << endl;
  
  return 0;
}
