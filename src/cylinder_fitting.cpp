
// Author(s): Luca Gemma

#include <string>
#include <sstream>
#include <stdlib.h>
#include <ios>
#include <iostream>


// ROS headers
#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

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

#include <pcl/sample_consensus/sac_model_cylinder.h>

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

#include "marker_generator.h"
#include "visual_perception/TabletopSegmentation.h"
#include "visual_perception/CylinderFitting.h"
#include "visual_perception/Cylinder.h" 


//#include <pcl/impl/point_types.hpp>
//#include <pcl/visualization/pcl_visualizer.h>



#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
//#include <pcl/features/don.h>


//using namespace pcl;
//using namespace std;












namespace visual_perception {

class CylinderFitter
{ //typedef pcl::PointXYZ    Point;
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;
  
private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher cylinderfitting_pub_ ;
  //! Service server for object detection
  ros::ServiceServer fitting_cylinder_srv_;

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

  //! Clouds are transformed into this frame before processing; leave empty if clouds
  //! are to be processed in their original frame
  std::string processing_frame_;

  //! A tf transform listener
  tf::TransformListener listener_;

    //! A tf transform broadcaster
  tf::TransformBroadcaster broadcaster_;

   



  //------------------ Callbacks -------------------

  
  //! Callback for service calls
 bool serviceCallback(CylinderFitting::Request &request, CylinderFitting::Response &response);
  
//------------------- Complete processing -----
// //! Publishes rviz markers for the given no cylinder clusters
  template <class PointCloudType>
  void publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header);
  

 public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
 CylinderFitter(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
    num_markers_published_ = 1;
    current_marker_id_ = 1;
    cylinderfitting_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

    fitting_cylinder_srv_ = nh_.advertiseService(nh_.resolveName("cylinder_fitting_srv"), &CylinderFitter::serviceCallback, this);
    
   priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 300);
   priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.01);
   priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
  }


  //! Empty stub
  ~CylinderFitter() {}
};



template <class PointCloudType>
void CylinderFitter::publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header)
{
  for (size_t i=0; i<clusters.size(); i++) 
  {
    visualization_msgs::Marker cloud_marker =  MarkerGenerator::getCloudMarker(clusters[i]);
    cloud_marker.header = cloud_header;
    cloud_marker.pose.orientation.w = 1;
    cloud_marker.ns = "cylinder_fitting_node";
    cloud_marker.id = current_marker_id_++;
    cylinderfitting_pub_.publish(cloud_marker);
  }
}












tf::Transform getCylinderTransform (pcl::ModelCoefficients coeffs)
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



bool CylinderFitter::serviceCallback(CylinderFitting::Request &request, CylinderFitting::Response &response)
{  
  
  int n_object;
  n_object=request.n_object;


  ros::Time start_time = ros::Time::now();
  ROS_INFO("Cylinder Fitting start !!!");
  std::vector<sensor_msgs::PointCloud> cluster;
  cluster=request.cluster;
  

 sensor_msgs::PointCloud2 converted_cloud;
 sensor_msgs::convertPointCloudToPointCloud2 (cluster[n_object], converted_cloud);
 ROS_INFO("Step 1 conversion from ros2pcl DONE !!!");
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
 pcl::fromROSMsg(converted_cloud, *cloud_ptr);

  ROS_INFO("Step 2 cloud_ptr DONE !!!");
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::VoxelGrid<Point> grid_, grid_objects_,grid_objects_no_cyl_;
  pcl::PassThrough<Point> pass_;
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConvexHull<Point> hull_;
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::EuclideanClusterExtraction<Point> pcl_cluster_;


// Clustering parameters
  pcl_cluster_.setClusterTolerance (cluster_distance_);
// pcl_cluster_.setMinClusterSize (min_cluster_size_);
  pcl_cluster_.setSearchMethod (clusters_tree_);
  grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
  grid_objects_no_cyl_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);



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
      response.result = response.NO_CYLINDER;

      //return;
    }
  else
  {

  
          



      ROS_INFO("Fit cylinders to each cluster in clusters2 IIIFFF the cluster is not empty");
      //  //Step 7: Fit cylinders to each cluster in clusters2 IIIFFF the cluster is not empty
      //Additional PCL objects
      KdTreePtr normals_cluster_tree_;
      pcl::NormalEstimation<Point, pcl::Normal> n3d_cluster_;  
      n3d_cluster_.setKSearch (10);  
      n3d_cluster_.setSearchMethod (normals_cluster_tree_);
      // Extract the cluster of interest
      pcl::PointCloud<Point>::Ptr cloud_cluster_ptr (new pcl::PointCloud<Point>);
      pcl::ExtractIndices<Point> extract_cluster_indices;
      ROS_INFO("Fit cylinders INIT Done");
      //extract_cluster_indices.setInputCloud(cloud_ptr);
    

      extract_cluster_indices.setInputCloud(cloud_objects_downsampled_ptr);
      ROS_INFO("Fit cylinders setInputCloud DONE");
      extract_cluster_indices.setIndices(boost::make_shared<const pcl::PointIndices> (clusters2[0]));
      extract_cluster_indices.filter(*cloud_cluster_ptr);
      ROS_INFO("Fit cylinders setIndices Done");
      extract_cluster_indices.filter(*cloud_ptr);
      // Estimate normals
      pcl::PointCloud<pcl::Normal>::Ptr cloud_cluster_normals_ptr (new pcl::PointCloud<pcl::Normal>);
       //n3d_cluster_.setInputCloud (cloud_ptr); 
      n3d_cluster_.setInputCloud (cloud_cluster_ptr);
      n3d_cluster_.compute (*cloud_cluster_normals_ptr);
      ROS_INFO("Step 2 Cylinder done");

      // Fit a cylinder to the cluster
      pcl::SampleConsensusModelCylinder<Point, pcl::Normal>::Ptr cyl_model_ (new pcl::SampleConsensusModelCylinder<Point, pcl::Normal> (cloud_ptr));
      cyl_model_->setInputNormals(cloud_cluster_normals_ptr);
      
      //cyl_model_->setInputNormals(cloud_cluster_normals_ptr);
      ROS_INFO("WAYTING FOR FITTING !!!"); 
      pcl::RandomSampleConsensus<Point> ransac (cyl_model_,1);
      
      // First, have a very rough approx of the cylinder
      ransac.computeModel();
      Eigen::VectorXf cyl_coeff;
      ransac.getModelCoefficients (cyl_coeff);

      if (cyl_coeff.size () <=3) 
       
      {
        ROS_INFO("Failed to fit a cylinder to the cluster");
        //response.result = response.NO_CYLINDER; //TODO change the response message to NO_CYLINDER
        //return;
      }
      else
      { response.result=response.SUCCESS;
        ROS_INFO(" Now, try to do a better fit, get the inliers");
        std::vector<int> cyl_inliers;
          ransac.getInliers (cyl_inliers);

          ROS_INFO("Give the previous coeff as an initial guess and optimize to fit to the inliers");
          Eigen::VectorXf cyl_coeff_optimized;
          cyl_model_-> optimizeModelCoefficients(cyl_inliers, cyl_coeff, cyl_coeff_optimized);

  //        ROS_INFO ("[ObjectFitter::input_callback] Model coefficients optimized are: Point in axis [%f %f %f], Axis [%f %f %f], Radius [%f].", cyl_coeff_optimized[0], cyl_coeff_optimized[1], cyl_coeff_optimized[2], cyl_coeff_optimized[3], cyl_coeff_optimized[4], cyl_coeff_optimized[5], cyl_coeff_optimized[6]);
        
         ROS_INFO("Step 3 Cylinder done");

  //        //Project the inliers into the cylinder model to obtain the cylinder height and the center of the cylinder.
        
          // Hard code to convert from Eigen::VecctorXf to pcl::ModelCoefficients and pcl::PointIndices  
          pcl::PointIndices::Ptr cylinder_inliers_ptr (new pcl::PointIndices); 
          cylinder_inliers_ptr->indices.resize((int)cyl_inliers.size());
 
         pcl::ModelCoefficients::Ptr cylinder_coefficients_ptr (new pcl::ModelCoefficients);
         cylinder_coefficients_ptr->values.resize(cyl_coeff_optimized.size());
        
         for (int i = 0; i < cyl_coeff_optimized.size(); i++)
         {   
           cylinder_coefficients_ptr->values[i] = cyl_coeff_optimized[i];
         }
         
         for (int i = 0; i < (int)cyl_inliers.size(); i++)
         {
           cylinder_inliers_ptr->indices[i] = cyl_inliers[i];
           //ROS_INFO("%d",cyl_inliers[i]);
         }
         

         
  //        // ROS_INFO("cyl_coeff_optimized.size() %d", (int)cyl_coeff_optimized.size());
  //        // ROS_INFO("cylinder_coefficients_ptr->values[0] %f", cylinder_coefficients_ptr->values[0]);
         ROS_INFO("cyl_inliers.size() %d", (int)cyl_inliers.size());
  //        // ROS_INFO("cylinder_inliers_ptr->indices.size() %d", (int)cylinder_inliers_ptr->indices.size());

  //        // ROS_INFO ("[TableObjectDetector::input_callback] Success in converting to pcl:: Model has with %d inliers: [%f %f %f %f %f %f %f].", (int)cylinder_inliers_ptr->indices.size(), cylinder_coefficients_ptr->values[0], cylinder_coefficients_ptr->values[1], cylinder_coefficients_ptr->values[2], cylinder_coefficients_ptr->values[3], cylinder_coefficients_ptr->values[4], cylinder_coefficients_ptr->values[5], cylinder_coefficients_ptr->values[6]);

  //        //// Project the cylinder inliers on the cylinder
  //        // pcl::PointCloud<Point>::Ptr cylinder_projected_ptr (new pcl::PointCloud<Point>); 
  //        //pcl::ProjectInliers<Point> proj_cyl_;
  //        //proj_cyl_.setModelType (pcl::SACMODEL_CYLINDER);
  //        //proj_cyl_.setInputCloud (cloud_cluster_ptr);
  //        //proj_cyl_.setIndices (cylinder_inliers_ptr);
  //        //  proj_cyl_.setModelCoefficients (cylinder_coefficients_ptr);
  //        //  proj_cyl_.filter (*cylinder_projected_ptr);


         // Get Cylinder transform, assumes cylinder coefficients are of the form axis = (x,y,z) + s*(ax, ay, az) and radius r.
         tf::Transform cylinder_trans;
         cylinder_trans = getCylinderTransform(*cylinder_coefficients_ptr);
        
         ROS_INFO ("[CylinderFitter::input_callback] Success in computing the cylinder transformation with 7 elements (pos, quat).");

         std::vector<float> cyl_lambdas((int)cloud_cluster_ptr->size()-1);

         // put the coefficients in a more readable form for computations
         double x = cyl_coeff_optimized[0], y = cyl_coeff_optimized[1], z = cyl_coeff_optimized[2], ax = cyl_coeff_optimized[3], ay = cyl_coeff_optimized[4], az = cyl_coeff_optimized[5];
         cyl_coeff_optimized[6]=fabs(cyl_coeff_optimized[6]);
         //float inv_norm;

         tf::Vector3 p0(x,y,z); // position of the axis
         tf::Vector3 w(ax, ay, az); // direcction of the axis
         tf::Vector3 p; // generic point in the cluster to be read from the cloud
         tf::Vector3 p0_corrected; // center of the cylinder

         ROS_INFO("The cloud has %d points", (int)cloud_cluster_ptr->size());
         // project all points in the cluster onto the cylinder axis
         for (int i = 0; i < (int)cloud_cluster_ptr->size()-1; i++)
         {
           Point point_in_cluster = cloud_cluster_ptr->at(i);    //ROS_INFO("Point [%d] has coordinates [%f, %f, %f]", i, point_in_cluster.x, point_in_cluster.y, point_in_cluster.z);
           p[0] = point_in_cluster.x;
           p[1] = point_in_cluster.y;
           p[2] = point_in_cluster.z;
           //inv_norm = pow(sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]),-1);
           cyl_lambdas[i] = w.dot(p-p0); ///w.norm2();
          }
          float lambda_max = *std::max_element(cyl_lambdas.begin(), cyl_lambdas.end());
          //float lambda_max = std::distance(cyl_lambdas.begin(),largest);
          float lambda_min = *std::min_element(cyl_lambdas.begin(), cyl_lambdas.end());
          //float lambda_min = std::distance(cyl_lambdas.begin(), smallest);
          float lambda_mean = 0.5*(lambda_max + lambda_min);
        
          float height = lambda_max - lambda_min;
        
          p0_corrected = p0 + lambda_mean*w;

         ROS_INFO("Center of the cylinder at [%f %f %f]", p0_corrected[0], p0_corrected[1], p0_corrected[2]);
         //ROS_INFO("Center of the cylinder at [%f %f %f]", p0[0], p0[1], p0[2]);
         ROS_INFO("Lambda_min %f, Lambda_max %f and Lambda_mean %f", lambda_min, lambda_max, lambda_mean);
        
         //tf::Vector3 w(ax, ay, az);
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
         
        
         tf::Transform cyl_trans_;
         cyl_trans_.setOrigin(p0_corrected);
         cyl_trans_.setRotation(orientation);
         ros::Time now = ros::Time::now();
         broadcaster_.sendTransform(tf::StampedTransform(cyl_trans_, now, "camera_rgb_optical_frame", "cylinder"));
        
        tf::StampedTransform cylinder_2_twist_stamped;
          
        listener_.waitForTransform("cylinder", "left_arm_7_link", now, ros::Duration(4));
        listener_.lookupTransform("cylinder", "left_arm_7_link", now, cylinder_2_twist_stamped);


         //broadcaster_.sendTransform(tf::StampedTransform(cyl_trans_, ros::Time::now(), "calib_kimp_right_arm_base_link", "cylinder"));

         tf::Transform cyl_in_base;
         tf::Transform base_2_camera;
         tf::StampedTransform base_2_camera_stamped;
        
         ros::Time now2 = ros::Time::now();   
         listener_.waitForTransform("world_link", "camera_rgb_optical_frame", now2, ros::Duration(4));
         listener_.lookupTransform("world_link", "camera_rgb_optical_frame", now2, base_2_camera_stamped);
        
         base_2_camera.setOrigin(base_2_camera_stamped.getOrigin());
         base_2_camera.setBasis(base_2_camera_stamped.getBasis());

         cyl_in_base = base_2_camera*cyl_trans_;


         // Convert cylinder raw data into the cylinder message
         Cylinder cylinder;
         if (cyl_coeff_optimized[6]<0.2)  
         {

           
     
           // Create the shape and the marker
           uint32_t cyl_shape = visualization_msgs::Marker::CYLINDER;
           visualization_msgs::Marker cyl_marker;
           // Set the marker type to CYLINDER
           cyl_marker.type = cyl_shape;

           // Set the frame ID and timestamp.  See the TF tutorials for information on these.
           //cyl_marker.header.frame_id = "/camera_rgb_optical_frame";
           cyl_marker.header.frame_id = "/world_link";

           cyl_marker.header.stamp = ros::Time::now();

           // Set the namespace and id for this marker.  This serves to create a unique ID
           // Any marker sent with the same namespace and id will overwrite the old one
           cyl_marker.ns = "cylinder_fitting_node";
           cyl_marker.id = n_object;

           // Set the marker action.  Options are ADD and DELETE
           cyl_marker.action = visualization_msgs::Marker::ADD;

           // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header.
           //  cyl_marker.pose.position.x = p0_corrected[0];
           //  cyl_marker.pose.position.y = p0_corrected[1];
           //  cyl_marker.pose.position.z = p0_corrected[2];
           //  cyl_marker.pose.orientation.x = orientation[0];
           //  cyl_marker.pose.orientation.y = orientation[1];
           //  cyl_marker.pose.orientation.z = orientation[2];
           //  cyl_marker.pose.orientation.w = orientation[3];

           tf::Vector3 origin = cyl_in_base.getOrigin();
           tf::Quaternion quaternion = cyl_in_base.getRotation();
           cyl_marker.pose.position.x = origin[0];
           cyl_marker.pose.position.y = origin[1];
           cyl_marker.pose.position.z = origin[2];
           cyl_marker.pose.orientation.x = quaternion[0];
           cyl_marker.pose.orientation.y = quaternion[1];
           cyl_marker.pose.orientation.z = quaternion[2];
           cyl_marker.pose.orientation.w = quaternion[3];
           
           //Controll cylinder fitting
           int n_inliers;
           n_inliers=cyl_model_->countWithinDistance(cyl_coeff_optimized,(double)0.009);
           float I;
           I= (float)n_inliers/(float)cloud_ptr->size();
           ROS_INFO("Indice di qualità del fitting %f",I);




           // Set the scale of the marker -- 1x1x1 here means 1m on a side
           cyl_marker.scale.x = 2*fabs(cyl_coeff_optimized[6]);
           cyl_marker.scale.y = 2*fabs(cyl_coeff_optimized[6]);
           cyl_marker.scale.z = height;

           // Set the color -- be sure to set alpha to something non-zero!
           cyl_marker.color.r = 1.0f-(double)I;
           cyl_marker.color.g = (double)I;
           cyl_marker.color.b = 0.0f;
           cyl_marker.color.a = 1.0;
           if ((double)I>(double)0.83)
           { 

           cylinder.r = cyl_coeff_optimized[6];
           cylinder.h = height;
           ROS_INFO("Radius of the cylinder %f and height %f", cyl_coeff_optimized[6], height);
           geometry_msgs::Pose cylinder_pose;
           // pusblish the cylinder in the camera frame
           //tf::poseTFToMsg(cyl_trans_, cylinder_pose);
           // pusblish the cylinder in the robot frame
           tf::poseTFToMsg(cyl_in_base, cylinder_pose);
           cylinder.pose.pose = cylinder_pose;
        
           //cylinder.pose.header = cloud.header;
        
           response.cylinder = cylinder;
             cyl_marker.lifetime = ros::Duration(10);
          
             std::string firstlevel ("/home/pacman/Projects/LUCAGEMMA/file_pcd/cylinder.txt");
             std::string thirdlevel  (".pcd");
             //std::string name ("/home/luca/file_pcd/cylinder_" + n_object + ".pcd");
             std::string name;
             std::string n_ob;
             std::ostringstream out;
             out << n_object ;
             n_ob = out.str();
             name = firstlevel+ n_ob +thirdlevel;
             pcl::PCDWriter writer;
             writer.writeASCII (name,*cloud_cluster_ptr);
             std::ofstream name_file_ptr;
             name_file_ptr.open ("/home/pacman/Projects/LUCAGEMMA/file_txt/cylinder_camera_link.txt");
             
             name_file_ptr << cyl_coeff_optimized[6] <<" ";
             name_file_ptr << height <<" ";

             for (size_t i=0; i<3; ++i)
                  name_file_ptr << p0_corrected[i] <<" ";


             for (size_t i=0; i<4; ++i)
                  name_file_ptr << orientation[i] <<" ";
                   
             name_file_ptr << "\n";
             name_file_ptr.close();   

             
             tf::Transform cylinder_2_twist;
             cylinder_2_twist.setOrigin(cylinder_2_twist_stamped.getOrigin());
             cylinder_2_twist.setBasis(cylinder_2_twist_stamped.getBasis());

             tf::Vector3 origin_twist = cylinder_2_twist.getOrigin();
             tf::Quaternion quaternion_twist = cylinder_2_twist.getRotation();

             std::ofstream name2_file_ptr;
             name2_file_ptr.open ("/home/pacman/Projects/LUCAGEMMA/file_txt/cylinder_twist_link.txt");
             
             name2_file_ptr << cyl_coeff_optimized[6] <<" ";
             name2_file_ptr << height <<" ";

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
               cyl_marker.lifetime = ros::Duration(0.5);
              }
            // Publish the marker
           cylinderfitting_pub_.publish(cyl_marker);
          

           response.I=I;
           ROS_INFO("controll cloud no inliers");
           pcl::ExtractIndices<Point> extract;
           extract.setInputCloud (cloud_ptr);
           extract.setIndices (cylinder_inliers_ptr);
           extract.setNegative (true);
           pcl::PointCloud<Point>::Ptr cloud_no_cylinder (new pcl::PointCloud<Point> ());
           extract.filter (*cloud_no_cylinder);

           ROS_INFO("Downsample the points"); 
           // ---[ Downsample the points
           pcl::PointCloud<Point>::Ptr cloud_objects_no_cyl_downsampled_ptr (new pcl::PointCloud<Point>); 
           grid_objects_no_cyl_.setInputCloud (cloud_no_cylinder);
           grid_objects_no_cyl_.filter (*cloud_objects_no_cyl_downsampled_ptr);
           
            ROS_INFO("Downsample the points DONE !!!"); 
            // Step 6: Split the objects into Euclidean clusters
            ROS_INFO ("WAYTING FOR: Split the objects into Euclidean clusters");
            std::vector<pcl::PointIndices> clusters2_no_cyl;
            pcl_cluster_.setInputCloud (cloud_objects_no_cyl_downsampled_ptr);
            pcl_cluster_.extract (clusters2_no_cyl);
            ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters2_no_cyl.size ());

            // ---[ Convert clusters into the PointCloud message
            std::vector<sensor_msgs::PointCloud> clusters_c_no_cyl;
            getClustersFromPointCloud2<Point> (*cloud_objects_no_cyl_downsampled_ptr, clusters2_no_cyl, clusters_c_no_cyl);
        
            ROS_INFO("Clusters converted");

           
            // FINAL, pusblish the markers
            publishClusterMarkers(clusters_c_no_cyl,  converted_cloud.header);
            

             if ((int)clusters2_no_cyl.size ()>1)
             {
              response.cluster_no_cyl=clusters_c_no_cyl;
             }
             else  response.result2 = response.NO_CLUSTER_NO_CYL;




          }
         
         ROS_INFO_STREAM("In total, cylinder fitting took " << ros::Time::now() - start_time << " seconds");
         return true;
      
        }
      }   
   }     










} //namespace visual_perception

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "cylinder_fitting_node");
  ros::NodeHandle nh;

  visual_perception::CylinderFitter node(nh);

  ros::spin();
  return 0;
}
