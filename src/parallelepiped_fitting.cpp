
// Author(s): Luca Gemma

#include <string>
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <ios>
#include <iostream>

// ROS headers
#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/transforms.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "std_msgs/String.h"

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLHeader.h>
#include <std_msgs/Header.h>
#include <pcl/io/ply_io.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>



#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/impl/angles.hpp>




#include "marker_generator.h"
#include "visual_perception/TabletopSegmentation.h"
#include "visual_perception/ParallelepipedFitting.h"
#include "visual_perception/Plane.h"
#include "visual_perception/Parallelepiped.h"
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


namespace visual_perception {

class ParallelepipedFitter
{ //typedef pcl::PointXYZ    Point;
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;
  
private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher parallelepipedfitting_pub_ ;
  //! Service server for object detection
  ros::ServiceServer fitting_parallelepiped_srv_;

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  //! Min distance between two clusters
  double cluster_distance_;
  //! Min number of points for a cluster
  int min_cluster_size_;
  //! Size of downsampling grid before performing clustering
  double clustering_voxel_size_;
   //! Positive or negative z is closer to the "up" direction in the processing frame?
  double up_direction_;
  bool flatten_plane_;
   //! How much the table gets padded in the horizontal direction
  double plane_padding_;

  //! Clouds are transformed into this frame before processing; leave empty if clouds
  //! are to be processed in their original frame
  std::string processing_frame_;

  //! A tf transform listener
  tf::TransformListener listener_;

    //! A tf transform broadcaster
  tf::TransformBroadcaster broadcaster_;

   



  //------------------ Callbacks -------------------

  
  //! Callback for service calls
 bool serviceCallback(ParallelepipedFitting::Request &request, ParallelepipedFitting::Response &response);
  
//------------------- Complete processing -----
// //! Publishes rviz markers for the given tabletop clusters

 //! Converts table convex hull into a triangle mesh to add to a Table messagetemplate <class PointCloudType>
 


 //template <class PointCloudType>
 //Plane ParallelepipedFitter::getPlane(std_msgs::Header cloud_header,const tf::Transform &table_plane_trans, const PointCloudType &table_points);



// //   template <class PointCloudType>
// //   void publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header);
//    //! Pull out and transform the convex hull points from a Table message
//   template <class PointCloudType>
//   bool tableMsgToPointCloud (Plane &plane, std::string frame_id, PointCloudType &plane_hull);
  

 public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
 ParallelepipedFitter(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
    num_markers_published_ = 1;
    current_marker_id_ = 1;
    parallelepipedfitting_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

    fitting_parallelepiped_srv_ = nh_.advertiseService(nh_.resolveName("parallelepiped_fitting_srv"), &ParallelepipedFitter::serviceCallback, this);
    

   //initialize operational flags
   priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 300);
   priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.03);
   priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
   priv_nh_.param<double>("up_direction", up_direction_, -1.0);
    priv_nh_.param<bool>("flatten_plane", flatten_plane_, false);

  }


  //! Empty stub
  ~ParallelepipedFitter() {}
};



/*template <class PointCloudType>
void CylinderFitting::publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header)
{
  for (size_t i=0; i<clusters.size(); i++) 
  {
    visualization_msgs::Marker cloud_marker =  MarkerGenerator::getCloudMarker(clusters[i]);
    cloud_marker.header = cloud_header;
    cloud_marker.pose.orientation.w = 1;
    cloud_marker.ns = "tabletop_node";
    cloud_marker.id = current_marker_id_++;
    marker_pub_.publish(cloud_marker);
  }
}*/

/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction, bool flatten_plane)
{
  ROS_ASSERT(coeffs.values.size() > 3);
  double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
  //asume plane coefficients are normalized
  tf::Vector3 position(-a*d, -b*d, -c*d);
  tf::Vector3 z(a, b, c);

  //if we are flattening the plane, make z just be (0,0,up_direction)
  if(flatten_plane)
  {
    ROS_INFO("flattening plane");
    z[0] = z[1] = 0;
    z[2] = up_direction;
  }
  else
  {
    //make sure z points "up"
    ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
    if ( z.dot( tf::Vector3(0, 0, up_direction) ) < 0)
    {
      z = -1.0 * z;
      ROS_INFO("flipped z");
    }
  }
    
  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  tf::Vector3 x(1, 0, 0);
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = tf::Vector3(0, 1, 0);
  tf::Vector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  tf::Matrix3x3 rotation;
  rotation[0] = x;  // x
  rotation[1] = y;  // y
  rotation[2] = z;  // z
  rotation = rotation.transpose();
  tf::Quaternion orientation;
  rotation.getRotation(orientation);
  ROS_DEBUG("in getPlaneTransform, x: %0.3f, %0.3f, %0.3f", x[0], x[1], x[2]);
  ROS_DEBUG("in getPlaneTransform, y: %0.3f, %0.3f, %0.3f", y[0], y[1], y[2]);
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);

  return tf::Transform(orientation, position);
}

template <typename PointT> 
bool getPlanePoints (const pcl::PointCloud<PointT> &table, 
         const tf::Transform& table_plane_trans,
         sensor_msgs::PointCloud &table_points)
{
  // Prepare the output
  table_points.header = pcl_conversions::fromPCL(table.header);
  table_points.points.resize (table.points.size ());
  for (size_t i = 0; i < table.points.size (); ++i)
  {
    table_points.points[i].x = table.points[i].x;
    table_points.points[i].y = table.points[i].y;
    table_points.points[i].z = table.points[i].z;
  }

  // Transform the data
  tf::TransformListener listener;
  // ros::Time table_stamp;
  // ros::Time now = ros::Time::now();
  // table_stamp.fromNSec(table.header.stamp*1000);
  // ROS_INFO("ROS TIME NOW %lu", now.toNSec()/1000);
  // ROS_INFO("TABLE HEADER STAMP %lu", table_stamp.toNSec()/1000);
  // table_stamp = pcl_conversions::fromPCL(table.header.stamp);
  tf::StampedTransform table_pose_frame(table_plane_trans, table_points.header.stamp, table.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
  {
    ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s", 
        table_points.header.frame_id.c_str(), error_msg.c_str());
    return false;
  }
  int current_try = 0, max_tries = 3;
  while (1)
  {
    bool transform_success = true;
    try
    {
      listener.transformPointCloud("table_frame", table_points, table_points);
    }
    catch (tf::TransformException ex)
    {
      transform_success = false;
      if ( ++current_try >= max_tries )
      {
        ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s", 
                  table_points.header.frame_id.c_str(), ex.what());
        return false;
      }
      //sleep a bit to give the listener a chance to get a new transform
      ros::Duration(0.1).sleep();
    }
    if (transform_success) break;
  }
  table_points.header.stamp = ros::Time::now();
  table_points.header.frame_id = "table_frame";
  return true;
}





tf::Transform getParallelepipedTransform (pcl::ModelCoefficients coeffs)
{
  // ROS_INFO("Size of coeffs: %d", (int)coeffs.values.size());
  // ROS_ASSERT(coeffs.values.size() > 8);
  double x = coeffs.values[0], y = coeffs.values[1], z = coeffs.values[2], ax = coeffs.values[3], ay = coeffs.values[4], az = coeffs.values[5];
  // r = coeffs.values[6]; the radius is not used

  // The position is directly obtained from the coefficients, and will be corrected later
  tf::Vector3 position(x,y,z);
  
  // w is the axis of the cylinder which will be aligned with the z reference frame of the cylinder
  tf::Vector3 w(ax, ay, az);
  tf::Vector3 u(1, 0, 0);
  if ( fabs(w.dot(u)) > 1.0 - 1.0e-4) u = tf::Vector3(0, 1, 0);
  tf::Vector3 v = w.cross(u).normalized();
  u = v.cross(w).normalized();
  tf::Matrix3x3 rotation;
  rotation[0] = u;  // x
  rotation[1] = v;  // y
  rotation[2] = w;  // z
  rotation = rotation.transpose();
  tf::Quaternion orientation;
  rotation.getRotation(orientation);

  // Compose the transformation and return it
  return tf::Transform(orientation, position);
}



template <typename PointT> void
getClustersFromPointCloud2 (const pcl::PointCloud<PointT> &cloud_objects,           
          const std::vector<pcl::PointIndices> &clusters2, 
          std::vector<sensor_msgs::PointCloud> &clusters)
{
  clusters.resize (clusters2.size ());
  for (size_t i = 0; i < clusters2.size (); ++i)
  {
    pcl::PointCloud<PointT> cloud_cluster;
    pcl::copyPointCloud(cloud_objects, clusters2[i], cloud_cluster);
   sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg( cloud_cluster, pc2 ); 
    sensor_msgs::convertPointCloud2ToPointCloud (pc2, clusters[i]);  
  }
}



bool ParallelepipedFitter::serviceCallback(ParallelepipedFitting::Request &request, ParallelepipedFitting::Response &response)
{  
  
  int n_object;
  n_object=request.n_object;
  Table table;
  table=request.table;
  ROS_INFO("Parallelepiped Fitting start !!!");
  ROS_INFO("Table:%f,%f,%f,%f,%f,%f",table.x_min,table.x_max,table.y_min,table.y_max,table.z_min,table.z_max);


  ros::Time start_time = ros::Time::now();
  
  std::vector<sensor_msgs::PointCloud> cluster;
  cluster=request.cluster;
  

 sensor_msgs::PointCloud2 converted_cloud;
 sensor_msgs::convertPointCloudToPointCloud2 (cluster[n_object], converted_cloud);
 ROS_INFO("Step 1 conversion from ros2pcl DONE !!!");
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
 pcl::fromROSMsg(converted_cloud, *cloud_ptr);
 // sensor_msgs::PointCloud2 cluster ;
  // sensor_msgs::convertPointCloud2ToPointCloud (cluster,request.cluster);
  // ROS_INFO("Step 1 conversion from ros2pcl DONE !!!");
  // pcl::PointCloud<Point>::Ptr cloud_ptr (new pcl::PointCloud<Point>); 
  // pcl::fromROSMsg(cluster, *cloud_ptr);
  ROS_INFO("Step 2 cloud_ptr DONE !!!");
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::VoxelGrid<Point> grid_, grid_objects_;
  pcl::PassThrough<Point> pass_;
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConvexHull<Point> hull_;
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::EuclideanClusterExtraction<Point> pcl_cluster_;
  pcl::PointCloud<Point>::Ptr plane_hull_1_ptr (new pcl::PointCloud<Point>);
  //pcl::PointCloud<Point>::Ptr plane_hull_2_ptr (new pcl::PointCloud<Point>);
  //pcl::PointCloud<Point>::Ptr plane_hull_3_ptr (new pcl::PointCloud<Point>);


// Clustering parameters
  pcl_cluster_.setClusterTolerance (cluster_distance_);
// pcl_cluster_.setMinClusterSize (min_cluster_size_);
  pcl_cluster_.setSearchMethod (clusters_tree_);
  grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);



  ROS_INFO("Init Done!");
 
  ROS_INFO("Downsample the points"); 
  // ---[ Downsample the points
  pcl::PointCloud<Point>::Ptr cloud_objects_downsampled_ptr (new pcl::PointCloud<Point>); 
  grid_objects_.setInputCloud (cloud_ptr);
  grid_objects_.filter (*cloud_objects_downsampled_ptr);
  ROS_INFO("Downsample the points DONE !!!"); 
  // Step 6: Split the objects into Euclidean clusters
  ROS_INFO ("WAYTING FOR: Split the objects into Euclidean clusters");
  std::vector<pcl::PointIndices> clusters2;
  pcl_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
  pcl_cluster_.extract (clusters2);
  ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters2.size ());

  // ---[ Convert clusters into the PointCloud message
  std::vector<sensor_msgs::PointCloud> clusters_c;
  getClustersFromPointCloud2<Point> (*cloud_objects_downsampled_ptr, clusters2, clusters_c);
  //getClustersFromPointCloud2<Point> (*cloud_ptr, clusters2, clusters_c);
  ROS_INFO("Clusters converted");


  // FINAL, pusblish the markers
  //publishClusterMarkers(clusters_c, cloud.header);
    
  if ((int)clusters2.size () == 0) 
    
    {
      ROS_INFO("NO object on the table");
      response.result = response.NO_PARALLELEPIPED;

      //return;
    }
  else
  {   
    ROS_INFO("Fit Parallelepiped to each cluster in clusters2 IIIFFF the cluster is not empty");
   //  //Step 7: Fit Parallelepiped to each cluster in clusters2 IIIFFF the cluster is not empty
   //Additional PCL objects
   KdTreePtr normals_cluster_tree_;
   pcl::NormalEstimation<Point, pcl::Normal> n3d_cluster_;  
   n3d_cluster_.setKSearch (10);  
   n3d_cluster_.setSearchMethod (normals_cluster_tree_);
   // Extract the cluster of interest
   pcl::PointCloud<Point>::Ptr cloud_cluster_ptr (new pcl::PointCloud<Point>);
   pcl::ExtractIndices<Point> extract_cluster_indices;
   ROS_INFO("Fit Parallelepiped INIT Done");
   //extract_cluster_indices.setInputCloud(cloud_ptr);
      

   extract_cluster_indices.setInputCloud(cloud_objects_downsampled_ptr);
   ROS_INFO("Fit Parallelepiped setInputCloud DONE");
   extract_cluster_indices.setIndices(boost::make_shared<const pcl::PointIndices> (clusters2[0]));
   extract_cluster_indices.filter(*cloud_cluster_ptr);
   ROS_INFO("Fit Parallelepiped setIndices Done");
   extract_cluster_indices.filter(*cloud_ptr);
   // Estimate normals
   pcl::PointCloud<pcl::Normal>::Ptr cloud_cluster_normals_ptr (new pcl::PointCloud<pcl::Normal>);
   //n3d_cluster_.setInputCloud (cloud_ptr); 
   n3d_cluster_.setInputCloud (cloud_cluster_ptr);
   n3d_cluster_.compute (*cloud_cluster_normals_ptr);
   ROS_INFO("Step 2 Parallelepiped done");
   // // Fit a Parallelepiped to the cluster
   pcl::SampleConsensusModelParallelPlane<Point>::Ptr plane_model1_ (new pcl::SampleConsensusModelParallelPlane<Point> (cloud_ptr));
   plane_model1_->setAxis (Eigen::Vector3f (1.0, 0.0, 0.0));
   plane_model1_->setEpsAngle (pcl::deg2rad (2.5));

      //pcl::SampleConsensusModelParallelPlane<Point>::Ptr plane_model2_ (new pcl::SampleConsensusModelParallelPlane<Point> (cloud_ptr));
      //plane_model2_->setAxis (Eigen::Vector3f (0.0, 1.0, 0.0));
      //plane_model2_->setEpsAngle (pcl::deg2rad (5.8));

      //pcl::SampleConsensusModelParallelPlane<Point>::Ptr plane_model3_ (new pcl::SampleConsensusModelParallelPlane<Point> (cloud_ptr));
      //plane_model3_->setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
      //plane_model3_->setEpsAngle (pcl::deg2rad (5.8));


      ///// P R O V A ///////

      // double z_min = 0., z_max = 0.05; // we want the points above the plane, no farther than 5 cm from the surface
      // pcl::PointCloud<Point>::Ptr hull_points (new pcl::PointCloud<Point> ());
      // pcl::ConvexHull<Point> hull;
      // // hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
      // hull.setInputCloud (*cloud_ptr);
      // hull.reconstruct (hull_points);
      // if (hull.getDimension () == 2)
      // {
      //   pcl::ExtractPolygonalPrismData<Point> prism;
      //   prism.setInputCloud (point_cloud);
      //   prism.setInputPlanarHull (hull_points);
      //   prism.setHeightLimits (z_min, z_max);
      //   prism.segment (cloud_indices);
      // }
      // else
      //   ROS_INFO("The input cloud does not represent a planar surface.\n");

      
   ROS_INFO("WAYTING FOR FITTING !!!"); 
   pcl::RandomSampleConsensus<Point> ransac1 (plane_model1_,1);
   //pcl::RandomSampleConsensus<Point> ransac2 (plane_model2_,1);
   //pcl::RandomSampleConsensus<Point> ransac3 (plane_model3_,1);
      
   // First, have a very rough approx of the plane
   ransac1.computeModel();
   //ransac2.computeModel();
   //ransac3.computeModel();

   Eigen::VectorXf pl_coeff_1;
   //Eigen::VectorXf pl_coeff_2;
   //Eigen::VectorXf pl_coeff_3;


   ransac1.getModelCoefficients (pl_coeff_1);
   //ransac2.getModelCoefficients (pl_coeff_2);
   //ransac3.getModelCoefficients (pl_coeff_3);

   if (pl_coeff_1.size () <=3) 
       
    {
      ROS_INFO("Failed to fit a plane to the cluster");
      //response.result = response.NO_CYLINDER; //TODO change the response message to NO_CYLINDER
      //return;
        
    }

    else
    { response.result=response.SUCCESS;
     ROS_INFO(" Now, try to do a better fit, get the inliers");
     std::vector<int> plane_inliers_1;
     ransac1.getInliers (plane_inliers_1);

     ROS_INFO("Give the previous coeff as an initial guess and optimize to fit to the inliers");
     Eigen::VectorXf plane_coeff_1_optimized;
     plane_model1_-> optimizeModelCoefficients(plane_inliers_1, pl_coeff_1, plane_coeff_1_optimized);

     ROS_INFO ("[ObjectFitter::input_callback] Plane Model coefficients optimized are: [%f %f %f %f].", plane_coeff_1_optimized[0], plane_coeff_1_optimized[1], plane_coeff_1_optimized[2], plane_coeff_1_optimized[3]);
     ROS_INFO("Step 3 Plane_1 done");

     //Project the inliers into the cylinder model to obtain the cylinder height and the center of the cylinder.
     // Hard code to convert from Eigen::VecctorXf to pcl::ModelCoefficients and pcl::PointIndices  
     pcl::PointIndices::Ptr plane_inliers_1_ptr (new pcl::PointIndices); 
     plane_inliers_1_ptr->indices.resize((int)plane_inliers_1.size());
           
     pcl::ModelCoefficients::Ptr plane_1_coefficients_ptr (new pcl::ModelCoefficients);
     plane_1_coefficients_ptr->values.resize(plane_coeff_1_optimized.size());
        
     for (int i = 0; i < plane_coeff_1_optimized.size(); i++)
     {   
       plane_1_coefficients_ptr->values[i] = plane_coeff_1_optimized[i];
     }
            
     for (int i = 0; i < (int)plane_inliers_1.size(); i++)
      {
        plane_inliers_1_ptr->indices[i] = plane_inliers_1[i];
             
      }
     // Step 4 : Project the table inliers on the table
     pcl::PointCloud<Point>::Ptr plane_projected_1_ptr (new pcl::PointCloud<Point>); 
     proj_.setInputCloud (cloud_objects_downsampled_ptr);
     proj_.setIndices (plane_inliers_1_ptr);
     proj_.setModelCoefficients (plane_1_coefficients_ptr);
     proj_.filter (*plane_projected_1_ptr);

     //          // Get Plane transform 
     tf::Transform plane_1_trans;
     tf::Transform plane_1_trans_flat;
     sensor_msgs::PointCloud plane_1_points;
     sensor_msgs::PointCloud plane_hull_1_points;


     plane_1_trans = getPlaneTransform(*plane_1_coefficients_ptr,up_direction_,false);
     hull_.setInputCloud (plane_projected_1_ptr);
     hull_.reconstruct (*plane_hull_1_ptr);
     plane_1_trans_flat = getPlaneTransform (*plane_1_coefficients_ptr, up_direction_, true);
     tf::Vector3 new_plane_pos;
     double avg_x, avg_y, avg_z;
     avg_x = avg_y = avg_z = 0;
     for (size_t i=0; i<plane_projected_1_ptr->points.size(); i++)
     {
       avg_x += plane_projected_1_ptr->points[i].x;
       avg_y += plane_projected_1_ptr->points[i].y;
       avg_z += plane_projected_1_ptr->points[i].z;
     }
     avg_x /= plane_projected_1_ptr->points.size();
     avg_y /= plane_projected_1_ptr->points.size();
     avg_z /= plane_projected_1_ptr->points.size();
     ROS_INFO("average x,y,z = (%.5f, %.5f, %.5f)", avg_x, avg_y, avg_z);

     // place the new plane frame in the center of the convex hull
     new_plane_pos[0] = avg_x;
     new_plane_pos[1] = avg_y;
     new_plane_pos[2] = avg_z;
     plane_1_trans.setOrigin(new_plane_pos);


     // shift the non-flat plane frame to the center of the convex hull as well
     plane_1_trans_flat.setOrigin(new_plane_pos);
           


            
     ROS_INFO ("[PlaneFitter::input_callback] Success in computing the plane transformation with 7 elements (pos, quat).");
     tf::Vector3 origin_1 = plane_1_trans.getOrigin();
     tf::Quaternion quaternion_1 = plane_1_trans.getRotation();
     ROS_INFO("Center of the plane at [%f %f %f]", origin_1[0], origin_1[1], origin_1[2]);
     ros::Time now = ros::Time::now();
     // broadcaster_.sendTransform(tf::StampedTransform(plane_1_trans_flat, ros::Time::now(), "camera_rgb_optical_frame", "plane_1_trans_flat"));
     broadcaster_.sendTransform(tf::StampedTransform(plane_1_trans, now, "camera_rgb_optical_frame", "plane_1"));
     tf::StampedTransform paralle_2_twist_stamped;
            
     listener_.waitForTransform("plane_1", "left_arm_7_link", now, ros::Duration(4));
     listener_.lookupTransform("plane_1", "left_arm_7_link", now, paralle_2_twist_stamped);

     tf::Transform plane_1_in_base;
     tf::Transform base_2_camera;
     tf::StampedTransform base_2_camera_stamped;
        
     ros::Time now2 = ros::Time::now();
     listener_.waitForTransform("world_link", "camera_rgb_optical_frame", now2, ros::Duration(4));
     listener_.lookupTransform("world_link", "camera_rgb_optical_frame", now2, base_2_camera_stamped);
        
     base_2_camera.setOrigin(base_2_camera_stamped.getOrigin());
     base_2_camera.setBasis(base_2_camera_stamped.getBasis());

     plane_1_in_base = base_2_camera*plane_1_trans;

     sensor_msgs::PointCloud plane_points;
            
            
     if (getPlanePoints<Point> (*plane_projected_1_ptr, plane_1_trans, plane_points))
     {
       Plane plane;
       //get the extents of the table
       if (!plane_1_points.points.empty()) 
        {
         plane.x_min = plane_points.points[0].x;
         plane.x_max = plane_points.points[0].x;
         plane.y_min = plane_points.points[0].y;
         plane.y_max = plane_points.points[0].y;
         plane.z_min = plane_points.points[0].z;
         plane.z_max = plane_points.points[0].z;
        }  
        for (size_t i=1; i<plane_points.points.size(); ++i) 
        {
         if (plane_points.points[i].x<plane.x_min && plane_points.points[i].x>-0.5) plane.x_min = plane_points.points[i].x;
         if (plane_points.points[i].x>plane.x_max && plane_points.points[i].x< 0.5) plane.x_max = plane_points.points[i].x;
         if (plane_points.points[i].y<plane.y_min && plane_points.points[i].y>-0.5) plane.y_min = plane_points.points[i].y;
         if (plane_points.points[i].y>plane.y_max && plane_points.points[i].y< 0.5) plane.y_max = plane_points.points[i].y;
         if (plane_points.points[i].z<plane.z_min && plane_points.points[i].z>-0.5) plane.z_min = plane_points.points[i].z;
         if (plane_points.points[i].z>plane.z_max && plane_points.points[i].z< 0.5) plane.z_max = plane_points.points[i].z;
        }
             

       ROS_INFO ("[PlaneFitter::input_callback] Success in computing plane.x_min,plane.x_max,plane.y_min,plane.y_max,plane.z_min,plane.z_max: %f,%f,%f,%f,%f,%f.",plane.x_min,plane.x_max,plane.y_min,plane.y_max,plane.z_min,plane.z_max);

       geometry_msgs::Pose plane_pose;
       // pusblish the cylinder in the camera frame
       tf::poseTFToMsg(plane_1_trans, plane_pose);
       // pusblish the cylinder in the robot frame
       //tf::poseTFToMsg(plane_1_in_base, plane_pose);
       plane.pose.pose = plane_pose;

       plane.x_min=plane.x_min;
       plane.x_max= plane.x_max;
       plane.y_min=plane.y_min;
       plane.y_max=plane.y_max;
       plane.z_min=plane.z_min;
       plane.z_max=plane.z_max;


       

       tf::Vector3 point_A(plane.x_max,plane.y_min,plane.z_max);
       tf::Vector3 point_B(plane.x_max,plane.y_max,plane.z_max) ;
       tf::Vector3 point_C(plane.x_min,plane.y_max,plane.z_max);

       float height = sqrt((plane.x_max-plane.x_min)*(plane.x_max-plane.x_min)+(plane.y_max-plane.y_max)*(plane.y_max-plane.y_max)+(plane.z_max-plane.z_max)*(plane.z_max-plane.z_max));

       float width  = sqrt((plane.x_max-plane.x_max)*(plane.x_max-plane.x_max)+(plane.y_max-plane.y_min)*(plane.y_max-plane.y_min)+(plane.z_max-plane.z_max)*(plane.z_max-plane.z_max));

       float depth  = sqrt((plane.x_max-plane.x_max)*(plane.x_max-plane.x_max)+(plane.y_min-plane.y_min)*(plane.y_min-plane.y_min)+(plane.z_max-plane.z_min)*(plane.z_max-plane.z_min));
            
       ROS_INFO ("[ParallelepipedFitter::input_callback] Success in computing height,width,depth: %f,%f,%f.",height,width,depth);
       if(height<0.6 & width<0.6 & depth<0.6)
       {

       response.plane_1 = plane;

       //Parallelepiped parallelepiped;

       uint32_t cube_shape = visualization_msgs::Marker::CUBE;
       visualization_msgs::Marker cube_marker;
       // Set the marker type to arrow
       cube_marker.type = cube_shape;

       // // Set the frame ID and timestamp.  See the TF tutorials for information on these.
       // //cyl_marker.header.frame_id = "/camera_rgb_optical_frame";
       cube_marker.header.frame_id = "/world_link";

       cube_marker.header.stamp = ros::Time::now();

       // // Set the namespace and id for this marker.  This serves to create a unique ID
       // // Any marker sent with the same namespace and id will overwrite the old one
       cube_marker.ns = "parallelepiped_fitting_node";
       cube_marker.id = n_object;

       // // Set the marker action.  Options are ADD and DELETE
       cube_marker.action = visualization_msgs::Marker::ADD;

        

       tf::Vector3 origin = plane_1_in_base.getOrigin();
       tf::Quaternion quaternion = plane_1_in_base.getRotation();

           
       cube_marker.pose.position.x = origin[0];
       cube_marker.pose.position.y = origin[1];
       cube_marker.pose.position.z = origin[2];
       cube_marker.pose.orientation.x = quaternion[0];
       cube_marker.pose.orientation.y = quaternion[1];
       cube_marker.pose.orientation.z = quaternion[2];
       cube_marker.pose.orientation.w = quaternion[3];







          
                   
       //Controll plane fitting
       int n_inliers;
       n_inliers=plane_model1_->countWithinDistance(plane_coeff_1_optimized,(double)0.025);
       ROS_INFO("n_inliers del fitting %d",n_inliers);
       ROS_INFO("(int)cloud_ptr->size() %d",(int)cloud_ptr->size());

       float I;
       //I= (float)plane_inliers_1.size()/(float)cloud_ptr->size();
       I= (float)n_inliers/(float)cloud_ptr->size();
       ROS_INFO("Indice di qualità del fitting %f",I);




        
       // // Set the scale of the marker 
       cube_marker.scale.x = fabs(height);
       cube_marker.scale.y = fabs(width);
       cube_marker.scale.z = fabs(depth);

       // // Set the color -- be sure to set alpha to something non-zero!
       cube_marker.color.r = 1.0f-(double)I;;
       cube_marker.color.g = (double)I;
       cube_marker.color.b = 0.0f;
       cube_marker.color.a = 1.0;
       if((double)I>0.45)
       {
         cube_marker.lifetime = ros::Duration(10);
          std::string firstlevel ("/home/pacman/Projects/LUCAGEMMA/file_pcd/parallelepiped_");
          std::string thirdlevel  (".pcd");
          //std::string name ("/home/luca/file_pcd/parallelepiped_" + n_object + ".pcd");
          std::string name;
          std::string n_ob;
          std::ostringstream out;
          out << n_object ;
          n_ob = out.str();
          name = firstlevel+ n_ob +thirdlevel;
          pcl::PCDWriter writer;
          writer.writeASCII (name, *cloud_cluster_ptr, false ); 
          std::ofstream name_file_ptr;
          name_file_ptr.open ("/home/pacman/Projects/LUCAGEMMA/file_txt/parallelepiped_camera_link.txt");

          name_file_ptr << height <<" "<< width <<" "<< depth <<" ";
          for (int i = 0; i < plane_coeff_1_optimized.size(); i++)
            name_file_ptr << plane_coeff_1_optimized[i] <<" ";
          
          for (size_t i=0; i<3; ++i)
           name_file_ptr << new_plane_pos[i] <<" ";

           for (size_t i=0; i<4; ++i)
              name_file_ptr << quaternion_1[i] <<" ";
                   
          name_file_ptr << "\n";
          name_file_ptr.close();  

          tf::Transform paralle_2_twist;
          paralle_2_twist.setOrigin(paralle_2_twist_stamped.getOrigin());
          paralle_2_twist.setBasis(paralle_2_twist_stamped.getBasis());

          tf::Vector3 origin_twist = paralle_2_twist.getOrigin();
          tf::Quaternion quaternion_twist = paralle_2_twist.getRotation();

          std::ofstream name2_file_ptr;
          name2_file_ptr.open ("/home/pacman/Projects/LUCAGEMMA/file_txt/paralle_twist_link.txt");
             
          name2_file_ptr << height <<" "<< width <<" "<< depth <<" ";
          for (int i = 0; i < plane_coeff_1_optimized.size(); i++)
            name2_file_ptr << plane_coeff_1_optimized[i] <<" ";
          
            

          for (size_t i=0; i<3; ++i)
            name2_file_ptr << origin_twist[i] <<" ";


          for (size_t i=0; i<4; ++i)
            name2_file_ptr << quaternion_twist[i] <<" ";
                   
          name2_file_ptr << "\n";
          name2_file_ptr.close(); 

          ROS_INFO("Saved all data");

        }
        else
        {
          cube_marker.lifetime = ros::Duration(0.5); 
          ROS_INFO("FINAL, publish the markers");
          // // Publish the marker
          parallelepipedfitting_pub_.publish(cube_marker);
         } 
        }
      }  
    }
  }   
}     










} //namespace visual_perception

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "parallelepiped_fitting_node");
  ros::NodeHandle nh;

  visual_perception::ParallelepipedFitter node(nh);

  ros::spin();
  return 0;
}