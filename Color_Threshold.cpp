#include "Color_Threshold.h"

Color_Threshold::Color_Threshold(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr)
{
	input_ptr = cloud_ptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(*input_ptr,*tmp_ptr);
	output_ptr = tmp_ptr;
}

void Color_Threshold::apply(double sat_th, double val_th)
{
	//convert RGB to HSV
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr input_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::PointCloudXYZRGBtoXYZHSV(*input_ptr,*input_hsv);
	
	for (uint32_t i = 0; i<input_hsv->points.size(); ++i)
	{
		if ((input_hsv->points[i].s < sat_th) && (input_hsv->points[i].v > val_th))
		{
			output_ptr->points[i].r = 255;
			output_ptr->points[i].g = 255;
			output_ptr->points[i].b = 255;
		}
		else
		{
			output_ptr->points[i].r = 0;
			output_ptr->points[i].g = 0;
			output_ptr->points[i].b = 0;
		}
	}
	 
	 
	//ToDo: new ideas to improve
}  


pcl::PointCloud<pcl::PointXYZRGB>::Ptr& Color_Threshold::getOutput()
{
  return output_ptr;
}

void Color_Threshold::visualize()
{

  //input cloud
  pcl::visualization::PCLVisualizer viewer0("Parking floor");
	int v0(0);
	viewer0.setBackgroundColor(0.7,0.7,0.7,v0); // Set background to a dark grey
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> i_rgb(input_ptr); // Define R,G,B colors for the point cloud
	viewer0.addPointCloud(input_ptr,i_rgb,"clusters",v0);

	while(!viewer0.wasStopped())
	{
		viewer0.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	//output cloud
	pcl::visualization::PCLVisualizer viewer1("Threshold result");
	int v1(0);
	viewer1.setBackgroundColor(0.5,0.5,0.5,v1); // Set background to a dark grey
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> o_rgb(output_ptr); // Define R,G,B colors for the point cloud
	viewer1.addPointCloud(output_ptr,o_rgb,"output",v1);
    	
	while(!viewer1.wasStopped())
	{
		viewer1.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

Color_Threshold::~Color_Threshold()
{
  // free the memory allocated

}

