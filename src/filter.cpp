#include <iostream>

// ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

// Class prototype

// Global variables
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_by_sor;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_by_vg_sor;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented;

pcl::VoxelGrid<pcl::PointXYZ> vg_filter_;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

ros::Publisher pub_filtered_cloud;
ros::Publisher pub_segmented_cloud;

void FilterCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud)
{
	std::cout << "Aplying filters ... !" << std::endl;

	// First apply statistical outlier removal
	sor.setInputCloud (_cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (0.01);
	sor.filter (*cloud_filtered_by_sor);

	// Then gridify it
	/*vg_filter_.setInputCloud (cloud_filtered_by_sor);
	vg_filter_.setLeafSize (0.001f, 0.001f, 0.001f);
	vg_filter_.filter (*cloud_filtered_by_vg_sor);*/
	
	/*pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered_by_sor);
	
	std::vector<pcl::PointIndices> cluster_indices;
	ec.setClusterTolerance (0.01); // 2cm
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (50000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered_by_sor);
	ec.extract (cluster_indices);
	
	// Color each cluster and publish in a single cloud
	cloud_segmented->header = _cloud->header;
        cloud_segmented->width = 0;
	cloud_segmented->height = 1;
	cloud_segmented->is_dense = true;
	
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
	  std::uint8_t color = rand() % 256;
	  std::cout << "COLOR " << color << std::endl;
	  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	  {
	    pcl::PointXYZ pcl_pointXYZ = cloud_filtered_by_sor->points[*pit];
	    pcl::PointXYZRGB pcl_point;
	    pcl_point.x = pcl_pointXYZ.x;
	    pcl_point.y = pcl_pointXYZ.y;
	    pcl_point.z = pcl_pointXYZ.z;
	    pcl_point.r = color;
	    pcl_point.g = 0.0;
	    pcl_point.b = color;
	    cloud_segmented->points.push_back (pcl_point);
	    cloud_segmented->width = cloud_segmented->width + 1;
	  }
	}*/
	
	pub_filtered_cloud.publish(*cloud_filtered_by_sor);
	//pub_filtered_cloud.publish(*cloud_filtered_by_vg_sor);
	
	// pub_segmented_cloud.publish(*cloud_segmented);
}

int main (int argc, char** argv)
{

	//init ros
	ros::init(argc, argv, "filter_for_shapelization_node");
	ros::NodeHandle nh;
	
	ros::Subscriber sub_cloud = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/ensenso_nx/ensenso_cloud", 1, FilterCloudCallback);
	
	pub_filtered_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("filtered_cloud", 1);
	pub_segmented_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("segmented_cloud", 1);
	
	cloud_filtered_by_sor.reset(new pcl::PointCloud<pcl::PointXYZ>());
	cloud_filtered_by_vg_sor.reset(new pcl::PointCloud<pcl::PointXYZ>());
	cloud_segmented.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	ros::spin();

	return (0);
}
