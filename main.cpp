#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types_conversion.h>

//#include <pcl/filters/passthrough.h>

using namespace std;

int main()
{
  cout << "Welcome to the parking project" << endl;

	// Load PLY file
//	string fName = "../../PointCloud_Inglostadt_Theater2_10mm/pointcloud.ply";

//	string fName = "/media/fabio/Sandeep_2TB/Parkopedia-Exercise/PointCloud_Inglostadt_Theater_10mm/pointcloud.ply"; //1.4GB
	string fName = "/media/fabio/Sandeep_2TB/Parkopedia-Exercise/PointCloud_Inglostadt_Theater2_10mm/pointcloud.ply"; //1.3GB
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

	//apply a color threshold on floor to detect parking lines
	cout << "Applying a color threshold (manually)... " << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_bw(new pcl::PointCloud<pcl::PointXYZRGB>()); //black and white cloud
/*	
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
	
	//convert RGB to HSV
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr floor_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());;
	pcl::PointCloudXYZRGBtoXYZHSV(*floor_rgb,*floor_hsv);
	for (unsigned int i = 0; i<floor_hsv->points.size(); ++i)
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


	// Visualization
	cout << "Visualizing ... " << endl;
	pcl::visualization::PCLVisualizer viewer(" PCL Visualizer");
	int v1(0);
	viewer.createViewPort(0.0,0.0,0.5,1.0,v1);
	viewer.setBackgroundColor(0.5,0.5,0.5,v1);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> i_rgb(input_cloud);
	//viewer.addPointCloud(input_cloud,i_rgb,"input",v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> f_rgb(floor_rgb);
	viewer.addPointCloud(floor_rgb,f_rgb,"floor-rgb",v1);

	int v2(0);
	viewer.createViewPort(0.5,0.0,1.0,1.0,v2);
	viewer.setBackgroundColor(0.7,0.7,0.7,v2);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> f_bw(floor_bw);
	viewer.addPointCloud(floor_bw,f_bw,"floor-bw",v2);

	//add coordinate system
//	viewer.addCoordinateSystem(1.0);

	//add a sphere at origin
	pcl::PointXYZ op; //origin
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
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	cout << "Visualizing ... Done" << endl;
  
  return 0;
}
