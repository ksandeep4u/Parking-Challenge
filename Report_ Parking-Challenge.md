# Report: Parking-Challenge
# Author: Sandeep Katragadda

This report presents my approach to find the parking bays in the given point clouds. The clouds are given in .PLY format. Each point in the clouds has three dimensions X, Y and Z, and a color represented by R, G and B values.

## Pre-requisites
C++11 is the programming language used to achieve the goal. The libraries used are the open source Point Cloud Library (PCL 1.7), Visualization Toolkit (VTK) and Eigen. CMake 3.9.0 is used to compile the code.

## Loading the .PLY file
Each *<filePath/fileName>.ply* file is loaded using the pcl::io module as:

```C++=
pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::io::loadPLYFile("<filePath/fileName>.ply",*input_cloud);
```

## Pre-preprocessing: floor extraction
As the parking bays are always on the floor, the first step is to find the plane equation of the floor. Z = 0m represents the floor in all the four given clouds. As all the points on the floor have Z values close to zero, the points whose Z values are between -0.1m and 0.1m are considered as the floor which is our region of interest (ROI). Note that the considered thickness of the floor is 0.2m (20cm). This can be achieved using the *pcl::PassThrough<PointType>* filter. However, as there is a problem in linking to *pcl/filters* library (I guess it is a bug in PCL 1.7), an iterative check is performed on the Z values of all points in the *input_cloud* as:

```C++=
float thickness = 0.2;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
for(uint32_t i = 0; i<input_cloud->points.size(); ++i)
{
    if ((input_cloud->points[i].z > -thickness/2.0) && (input_cloud->points[i].z < thickness/2.0))
    {
        floor_cloud->push_back(input_cloud->points[i]);
    }
}
```

To reduce the processing time, the rest of the processing is done only on the floor.


## Approach 1: Parking line identification via color thresholding
In the first approach, a threshold on the color is used to identify the white parking lines of the floor. This is done in the HSV color space so the *floor_cloud* which is in the RGB color space is converted to HSV color space as:

```C++=
pcl::PointCloud<pcl::PointXYZHSV>::Ptr floor_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloudXYZRGBtoXYZHSV(*floor_cloud,*floor_hsv);
```

An upper threshold, 0.4, on the saturation (S value) and a lower threshold, 0.6, on the intensity (V value) are applied to check if a floor point color is close to white. All the points satisfying the thresholds are considered as white points and the remaining are black points. See the code below.

```C++=
for (uint32_t i=0; i<floor_hsv->points.size(); ++i)
{
    pcl::PointXYZI pt;
    pt.x = floor_hsv->points[i].x;
    pt.y = floor_hsv->points[i].y;
    pt.z = floor_hsv->points[i].z;
    if ((floor_hsv->points[i].s < sat_th) && (floor_hsv->points[i].v > val_th))
        pt.intensity = 255;
    else
    	pt.intensity = 0;
    floor_bw_cloud->push_back(pt);
}
```

The results of the floor extraction and the white part detection are shown here:

Floor and the it's white part of wolfsburg galerie: (PointCloud_5mm__wolfsburg_galerie_1b_01/pointcloud.ply)
<img src="https://i.imgur.com/RmSvv7j.png" width="350"/> <img src="https://i.imgur.com/IfkDSmF.png" width="350"/>

Floor and the it's white part of wolfsburg galerie:
(PointCloud_5mm__wolfsburg_galerie_1b_02/pointcloud.ply)
<img src="https://i.imgur.com/dtqj64H.png" width="350"/> <img src="https://i.imgur.com/Li1PfFj.png" width="350"/>

Floor and it's white part of Inglostadt Theater:
(PointCloud_Inglostadt_Theater_10mm/pointcloud.ply)
<img src="https://i.imgur.com/GhZVUgE.png" width="350"/> <img src="https://i.imgur.com/ZDDV4nJ.png" width="350"/>

Floor and it's white part of Inglostadt Theater:
(PointCloud_Inglostadt_Theater2_10mm/pointcloud.ply)
<img src="https://i.imgur.com/X2aFXON.png" width="350"/> <img src="https://i.imgur.com/20KyaVp.png" width="350"/>

Due to the illumination changes on the floor, the results do not show the parking bays clearly so a second approach based on color-based segmentation is used.



## Approach 2: Color-based segmentation
Each parking bay is surrounded by a white rectangle and the colors of the points in each rectangle are almost the same so in the second approach, color-based segmentation is performed on the floor using *pcl::RegionGrowingRGB* class. There are several parameters such as distance threshold, color thresholds and cluster size to tune during the segmentation. The parameters play crucial role in clustering. See the clusters of wolfsburg galerie. There is not even one bay detected. This is bacause the bays are not clearly separated.

While doing experiments, only a portion of the floor is used for quick processing. The points that are within a 12m radius from the centroid of the floor_cloud are selected. The centroid is computed using *pcl::computeCentroid()*. The points around the centroid are selected using the *radiusSearch()* function of the *pcl::kdTreeFLANN<PointType>* class. The selected regions are shown here:

Selected region of the floor of wolfsburg galerie: (PointCloud_5mm__wolfsburg_galerie_1b_01/pointcloud.ply)
![](https://i.imgur.com/JZLPcgr.png)

Selected region of the floor of wolfsburg galerie: (PointCloud_5mm__wolfsburg_galerie_1b_02/pointcloud.ply)
![](https://i.imgur.com/U1ZuEOx.png)

Selected region of the floor of Inglostadt Theater:
(PointCloud_Inglostadt_Theater_10mm/pointcloud.ply)
![](https://i.imgur.com/2d5fbG1.png)

Selected region of the floor of Inglostadt Theater:
(PointCloud_Inglostadt_Theater2_10mm/pointcloud.ply)
![](https://i.imgur.com/eEPSGdc.png)



The clusters of the segementation for wolfsburg galerie (PointCloud_5mm__wolfsburg_galerie_1b_01/pointcloud.ply) is shown here:
![](https://i.imgur.com/jToFXnc.png)

The clusters of the segementation for wolfsburg galerie (PointCloud_5mm__wolfsburg_galerie_1b_02/pointcloud.ply) is shown here:
![](https://i.imgur.com/ALV03nT.png)


The clusters of the segementation for Inglostadt Theater (PointCloud_Inglostadt_Theater_10mm/pointcloud.ply) is shown here:
![](https://i.imgur.com/WAIqXlE.png)

The clusters of the segementation for Inglostadt Theater (PointCloud_Inglostadt_Theater2_10mm/pointcloud.ply) is shown here:
![](https://i.imgur.com/ajzRYEY.png)



### Computing the bounding boxes of parking bays
If the clustering is done accurately, each cluster represents a parking bay. To visualize the parking bays, the bounding box of each cluster is computed using *pcl::MomentOfInertialEstimation<PointType>*. The parking bays need not be aligned with the coordinate axis so oriented bounding boxes (OBB) are computed instead of axis-aligned bounding boxes. The bounding box of each cluster *i* is a cuboid whose position and orientation are extracted by the function *pcl::MomentOfInertialEstimation<PointType>::getOBB()* as:

```C++=
pcl::PointXYZ min;
pcl::PointXYZ max;
pcl::PointXYZ pos;
Eigen::Matrix3f rot_mat;
pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::copyPointCloud(*floor_cloud,*tmp_cloud);
pcl::PointIndices::Ptr cl_ptr = boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(clusters[i]));

pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
feature_extractor.setInputCloud(tmp_cloud);
feature_extractor.setIndices(cl_ptr);
feature_extractor.compute();
feature_extractor.getOBB(min,max,pos,rot_mat);
```

The result of the segementation for wolfsburg galerie: (PointCloud_5mm__wolfsburg_galerie_1b_01/pointcloud.ply)
![](https://i.imgur.com/3Eo0jR9.jpg)

The result of the segementation for wolfsburg galerie: (PointCloud_5mm__wolfsburg_galerie_1b_02/pointcloud.ply)
![](https://i.imgur.com/lN0dIVi.png)


The result of the segementation for Inglostadt Theater (PointCloud_Inglostadt_Theater_10mm/pointcloud.ply) is shown here:
The bounding boxes (in white color):
![](https://i.imgur.com/4QZnDqM.png)

The result of the segementation for Inglostadt Theater (PointCloud_Inglostadt_Theater2_10mm/pointcloud.ply) is shown here:
The bounding boxes (in white color):
![](https://i.imgur.com/CAkt5gn.png)




### Refine clusters
Some of the clusters are not the parking bays so a refining step is needed. As parking bays have almost a constant size, each cluster size can be verified to check if it is a parking bay or not.
If the area of a cluster is between 4sqm and 18sqm the cluster is detected as a parking bay.

The result of the refinement for Inglostadt Theater (PointCloud_Inglostadt_Theater_10mm/pointcloud.ply) is shown here:
The bounding boxes (in white color):
![](https://i.imgur.com/pzne2Ij.png)

The result of the refinement for Inglostadt Theater (PointCloud_Inglostadt_Theater2_10mm/pointcloud.ply) is shown here:
The bounding boxes (in white color):
![](https://i.imgur.com/2VUk4J9.png)


## Visualization
To visualize the ouptput clouds, *pcl::visualization::PCLVisualizer* class is used as:

```C++=
pcl::visualization::PCLVisualizer viewer("Parking floor");
int v1(0);
viewer.setBackgroundColor(0.7,0.7,0.7,v1); // Set background to a dark grey
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> f_rgb(floor_cloud); // Define R,G,B colors for the point cloud
viewer.addPointCloud(floor_cloud,f_rgb,"floor",v1);
```

The bounding box of each cluster *i* is represented by a translation vector, a quaternion (orientation), width, height and depth. They are computed using the extracted OBB features and are drawn as:

```C++=
double width = max.x-min.x;
double height = max.y-min.y;
double depth = max.z-min.z;
Eigen::Vector3f trans_vec(pos.x,pos.y,pos.z);
Eigen::Quaternionf quat(rot_mat);
viewer.addCube(trans_vec,quat,width,height,depth,"box "+std::to_string(i),v1);
```

## Additional comments

- 3D edge detection is another possible approach to find the parking lines. It is provided by *pcl/features/organized_edge_detection* (supported by PCL 1.8). Other alternatives are *pcl::OrganizedEdgeBase* and *pcl::LineRGBD*. As all parking bays have similar features (shape), template matching can also be used.


- Due to the large amount of data to be processed, high-performance processors such as GPUs are required to implement and test the PCL algorithms.

