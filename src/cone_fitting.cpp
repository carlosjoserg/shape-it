
// Author(s): Luca Gemma

#include <string>
#include <sstream>
#include <stdlib.h>
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

#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/sample_consensus/sac_model_cone.h>

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
#include "visual_perception/ConeFitting.h"









namespace visual_perception {

class ConeFitter
{ //typedef pcl::PointXYZ    Point;
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;
  
private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher conefitting_pub_ ;
  //! Service server for object detection
  ros::ServiceServer fitting_cone_srv_;

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
 bool serviceCallback(ConeFitting::Request &request, ConeFitting::Response &response);


  
//------------------- Complete processing -----
// //! Publishes rviz markers for the given cone clusters
  template <class PointCloudType>
  void publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header);

 public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
 ConeFitter(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
    num_markers_published_ = 1;
    current_marker_id_ = 1;
    conefitting_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

    fitting_cone_srv_ = nh_.advertiseService(nh_.resolveName("cone_fitting_srv"), &ConeFitter::serviceCallback, this);
    

   //initialize operational flags
   priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 300);
   priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.03);
   priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
  }


  //! Empty stub
  ~ConeFitter() {}
};



template <class PointCloudType>
void ConeFitter::publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header)
{ 
  for (size_t i=0; i<clusters.size(); i++) 
  { 

    visualization_msgs::Marker cloud_marker =  MarkerGenerator::getCloudMarker(clusters[i]);
    cloud_marker.header = cloud_header;
    cloud_marker.pose.orientation.w = 1;
    cloud_marker.ns = "tabletop_node";
    cloud_marker.id = current_marker_id_++;
    conefitting_pub_ .publish(cloud_marker);
  }
}












tf::Transform getConeTransform (pcl::ModelCoefficients coeffs)
{
  // ROS_INFO("Size of coeffs: %d", (int)coeffs.values.size());
  // ROS_ASSERT(coeffs.values.size() > 8);
  double x = coeffs.values[0], y = coeffs.values[1], z = coeffs.values[2], ax = coeffs.values[3], ay = coeffs.values[4], az = coeffs.values[5];
  // r = coeffs.values[6]; the radius is not used

  // The position is directly obtained from the coefficients, and will be corrected later
  tf::Vector3 position(x,y,z);
  
  // w is the axis of the Cone which will be aligned with the z reference frame of the Cone
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



bool ConeFitter::serviceCallback(ConeFitting::Request &request, ConeFitting::Response &response)
{  
  
  int n_object;
  n_object=request.n_object;


  ros::Time start_time = ros::Time::now();
  ROS_INFO("Cone Fitting start !!!");
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
  //publishClusterMarkers(clusters_c, converted_cloud.header);
    
  if ((int)clusters2.size () == 0) 
    
    {
      ROS_INFO("NO object on the table");
      response.result = response.NO_CONE;

      //return;
    }
  else
    {   
      ROS_INFO("Fit Cone to each cluster in clusters2 IIIFFF the cluster is not empty");
  //  //Step 7: Fit cone to each cluster in clusters2 IIIFFF the cluster is not empty
      //Additional PCL objects
      KdTreePtr normals_cluster_tree_;
      pcl::NormalEstimation<Point, pcl::Normal> n3d_cluster_;  
      n3d_cluster_.setKSearch (10);  
      n3d_cluster_.setSearchMethod (normals_cluster_tree_);
      // Extract the cluster of interest
      pcl::PointCloud<Point>::Ptr cloud_cluster_ptr (new pcl::PointCloud<Point>);
      pcl::ExtractIndices<Point> extract_cluster_indices;
      ROS_INFO("Fit Cone INIT Done");
      //extract_cluster_indices.setInputCloud(cloud_ptr);
      

      extract_cluster_indices.setInputCloud(cloud_objects_downsampled_ptr);
      ROS_INFO("Fit Cone setInputCloud DONE");
      extract_cluster_indices.setIndices(boost::make_shared<const pcl::PointIndices> (clusters2[0]));
      extract_cluster_indices.filter(*cloud_cluster_ptr);
      ROS_INFO("Fit Cone setIndices Done");
      extract_cluster_indices.filter(*cloud_ptr);
      // Estimate normals
      pcl::PointCloud<pcl::Normal>::Ptr cloud_cluster_normals_ptr (new pcl::PointCloud<pcl::Normal>);
       //n3d_cluster_.setInputCloud (cloud_ptr); 
      n3d_cluster_.setInputCloud (cloud_cluster_ptr);
      n3d_cluster_.compute (*cloud_cluster_normals_ptr);
      ROS_INFO("Step 2 Cone done");

      // Fit a cylinder to the cluster
      pcl::SampleConsensusModelCone<Point, pcl::Normal>::Ptr cone_model_ (new pcl::SampleConsensusModelCone<Point, pcl::Normal> (cloud_ptr));
      //pcl::SampleConsensusModelCone<Point>::Ptr cone_model_ (new pcl::SampleConsensusModelCone<Point> (cloud_ptr));
      cone_model_->setInputNormals(cloud_cluster_normals_ptr);
      
      //cyl_model_->setInputNormals(cloud_cluster_normals_ptr);
      ROS_INFO("WAYTING FOR FITTING !!!"); 
      pcl::RandomSampleConsensus<Point> ransac (cone_model_,1);
      
      // First, have a very rough approx of the cylinder
      ransac.computeModel();
      Eigen::VectorXf cone_coeff;
      ransac.getModelCoefficients (cone_coeff);

        if (cone_coeff.size () <=3) 
       
        {
           ROS_INFO("Failed to fit a cone to the cluster");
           response.result = response.NO_CONE; //TODO change the response message to NO_CYLINDER
           //return;
         }
        else
        { response.result=response.SUCCESS;
          ROS_INFO(" Now, try to do a better fit, get the inliers");
          std::vector<int> cone_inliers;
          ransac.getInliers (cone_inliers);

          ROS_INFO("Give the previous coeff as an initial guess and optimize to fit to the inliers");
          Eigen::VectorXf cone_coeff_optimized;
          cone_model_-> optimizeModelCoefficients(cone_inliers, cone_coeff, cone_coeff_optimized);

  //        ROS_INFO ("[ObjectFitter::input_callback] Model coefficients optimized are: Point in axis [%f %f %f], Axis [%f %f %f], Radius [%f].", cyl_coeff_optimized[0], cyl_coeff_optimized[1], cyl_coeff_optimized[2], cyl_coeff_optimized[3], cyl_coeff_optimized[4], cyl_coeff_optimized[5], cyl_coeff_optimized[6]);
        
         ROS_INFO("Step 3 cone done");

  //        //Project the inliers into the cylinder model to obtain the cylinder height and the center of the cylinder.
        
          // Hard code to convert from Eigen::VecctorXf to pcl::ModelCoefficients and pcl::PointIndices  
          pcl::PointIndices::Ptr cone_inliers_ptr (new pcl::PointIndices); 
          cone_inliers_ptr->indices.resize((int)cone_inliers.size());
 
         pcl::ModelCoefficients::Ptr cone_coefficients_ptr (new pcl::ModelCoefficients);
         cone_coefficients_ptr->values.resize(cone_coeff_optimized.size());
        
         for (int i = 0; i < cone_coeff_optimized.size(); i++)
         {   
           cone_coefficients_ptr->values[i] = cone_coeff_optimized[i];
         }
         for (int i = 0; i < (int)cone_inliers.size(); i++)
         {
           cone_inliers_ptr->indices[i] = cone_inliers[i];
         }


         // // Project the cylinder inliers on the cylinder
         // pcl::PointCloud<Point>::Ptr cone_projected_ptr (new pcl::PointCloud<Point>); 
         // pcl::ProjectInliers<Point> proj_cone_;
         // proj_cone_.setModelType (pcl::SACMODEL_CONE);
         // proj_cone_.setInputCloud (cloud_cluster_ptr);
         // proj_cone_.setIndices (cone_inliers_ptr);
         //  proj_cone_.setModelCoefficients (cone_coefficients_ptr);
         //  proj_cone_.filter (*cone_projected_ptr);


         // Get cone transform, assumes cylinder coefficients are of the form axis = (x,y,z) + s*(ax, ay, az) and radius r.
         tf::Transform cone_trans;
       
         cone_trans = getConeTransform(*cone_coefficients_ptr);
        
         ROS_INFO ("[ConeFitter::input_callback] Success in computing the cone transformation with 7 elements (pos, quat).");

         std::vector<float> cone_lambdas((int)cloud_cluster_ptr->size()-1);
         // put the coefficients in a more readable form for computations
         double x = cone_coeff_optimized[0], y = cone_coeff_optimized[1], z = cone_coeff_optimized[2], ax = cone_coeff_optimized[3], ay = cone_coeff_optimized[4], az = cone_coeff_optimized[5];

         //float inv_norm;

         tf::Vector3 p0(x,y,z); // position of the axis
         tf::Vector3 w(ax, ay, az); // direcction of the axis
         tf::Vector3 p; // generic point in the cluster to be read from the cloud
         tf::Vector3 p0_corrected; // center of the cone

         ROS_INFO("The cloud has %d points", (int)cloud_cluster_ptr->size());
         // project all points in the cluster onto the cone axis
         for (int i = 0; i < (int)cloud_cluster_ptr->size()-1; i++)
         {
           Point point_in_cluster = cloud_cluster_ptr->at(i);    //ROS_INFO("Point [%d] has coordinates [%f, %f, %f]", i, point_in_cluster.x, point_in_cluster.y, point_in_cluster.z);
           p[0] = point_in_cluster.x;
           p[1] = point_in_cluster.y;
           p[2] = point_in_cluster.z;
           //inv_norm = pow(sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]),-1);
           cone_lambdas[i] = w.dot(p-p0); ///w.norm2();
          }
          float lambda_max = *std::max_element(cone_lambdas.begin(), cone_lambdas.end());
          //float lambda_max = std::distance(cyl_lambdas.begin(),largest);
          float lambda_min = *std::min_element(cone_lambdas.begin(), cone_lambdas.end());
          //float lambda_min = std::distance(cyl_lambdas.begin(), smallest);
          float lambda_mean = 0.5*(lambda_max + lambda_min);
        
          float height = lambda_max - lambda_min;
        
          p0_corrected = p0 + lambda_mean*w;
         

         ROS_INFO("Center of the cone at [%f %f %f]", p0_corrected[0], p0_corrected[1], p0_corrected[2]);
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
        
         tf::Transform cone_trans_;
         cone_trans_.setOrigin(p0_corrected);
         cone_trans_.setRotation(orientation);
         ros::Time now = ros::Time::now();
         broadcaster_.sendTransform(tf::StampedTransform(cone_trans_, now, "camera_rgb_optical_frame", "cone"));
         //broadcaster_.sendTransform(tf::StampedTransform(cyl_trans_, ros::Time::now(), "calib_kimp_right_arm_base_link", "cone"));
         
        tf::StampedTransform cone_2_twist_stamped;
            
        listener_.waitForTransform("cone", "left_arm_7_link", now, ros::Duration(4));
        listener_.lookupTransform("cone", "left_arm_7_link", now, cone_2_twist_stamped);



         tf::Transform cone_in_base;
         tf::Transform base_2_camera;
         tf::StampedTransform base_2_camera_stamped;
        
         ros::Time now2 = ros::Time::now();
         listener_.waitForTransform("world_link", "camera_rgb_optical_frame", now2, ros::Duration(4));
         listener_.lookupTransform("world_link", "camera_rgb_optical_frame", now2, base_2_camera_stamped);
        
         base_2_camera.setOrigin(base_2_camera_stamped.getOrigin());
         base_2_camera.setBasis(base_2_camera_stamped.getBasis());

         cone_in_base = base_2_camera*cone_trans_;

         // Convert cylinder raw data into the cylinder message
         
         if (cone_coeff_optimized[6]<0.2 & height<0.1)  
         { 
          
     
           // // Create the shape and the marker
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
           cube_marker.ns = "cone_fitting_node";
           cube_marker.id = n_object;

           // // Set the marker action.  Options are ADD and DELETE
           cube_marker.action = visualization_msgs::Marker::ADD;

        

           tf::Vector3 origin = cone_in_base.getOrigin();
           tf::Quaternion quaternion = cone_in_base.getRotation();

           
           cube_marker.pose.position.x = origin[0];
           cube_marker.pose.position.y = origin[1];
           cube_marker.pose.position.z = origin[2];
           cube_marker.pose.orientation.x = quaternion[0];
           cube_marker.pose.orientation.y = quaternion[1];
           cube_marker.pose.orientation.z = quaternion[2];
           cube_marker.pose.orientation.w = quaternion[3];







          
                   
           //Controll cone fitting
           int n_inliers;
           n_inliers=cone_model_->countWithinDistance(cone_coeff_optimized,(double)0.009);
           float I;
           I= (float)n_inliers/(float)cloud_ptr->size();
           ROS_INFO("Indice di qualità del fitting %f",I);



        
           // // Set the scale of the marker 
           cube_marker.scale.x = fabs(height*tan(cone_coeff_optimized[6]));
           cube_marker.scale.y = fabs(height*tan(cone_coeff_optimized[6]));
           cube_marker.scale.z = fabs(height);

           // // Set the color -- be sure to set alpha to something non-zero!
           cube_marker.color.r = 1.0f-(double)I;;
           cube_marker.color.g = (double)I;
           cube_marker.color.b = 0.0f;
           cube_marker.color.a = 1.0;
          if((double)I>0.85)
          { 
           Cone cone;
           cone.angle = cone_coeff_optimized[6];
           cone.h = height;
           ROS_INFO("Angle of the cone %f and height %f", cone_coeff_optimized[6], height);
           geometry_msgs::Pose cone_pose;
           // pusblish the cylinder in the camera frame
           tf::poseTFToMsg(cone_trans_, cone_pose);
           // pusblish the cylinder in the robot frame
           //tf::poseTFToMsg(cone_in_base,cone_pose);
           cone.pose.pose = cone_pose;
        
           //cylinder.pose.header = cloud.header;
        
           response.cone = cone;
           cube_marker.lifetime = ros::Duration(10);
            std::string firstlevel ("/home/pacman/Projects/LUCAGEMMA/file_pcd/cone_");
             std::string thirdlevel  (".pcd");
             //std::string name ("/home/luca/file_pcd/cylinder_" + n_object + ".pcd");
             std::string name;
             std::string n_ob;
             std::ostringstream out;
             out << n_object ;
             n_ob = out.str();
             name = firstlevel+ n_ob +thirdlevel;
           pcl::PCDWriter writer;
           writer.writeASCII (name, *cloud_cluster_ptr,false );
           std::ofstream name_file_ptr;
           name_file_ptr.open ("/home/pacman/Projects/LUCAGEMMA/file_txt/cone_camera_link.txt");
           name_file_ptr << height*tan(cone_coeff_optimized[6]) <<" ";
             name_file_ptr << height <<" ";

             for (size_t i=0; i<3; ++i)
                  name_file_ptr << p0_corrected[i] <<" ";


             for (size_t i=0; i<4; ++i)
                  name_file_ptr << orientation[i] <<" ";
                   
             name_file_ptr << "\n";
             name_file_ptr.close();  

             tf::Transform cone_2_twist;
             cone_2_twist.setOrigin(cone_2_twist_stamped.getOrigin());
             cone_2_twist.setBasis(cone_2_twist_stamped.getBasis());

             tf::Vector3 origin_twist = cone_2_twist.getOrigin();
             tf::Quaternion quaternion_twist = cone_2_twist.getRotation();

             std::ofstream name2_file_ptr;
             name2_file_ptr.open ("/home/pacman/Projects/LUCAGEMMA/file_txt/cone_twist_link.txt");
             
            name2_file_ptr << height*tan(cone_coeff_optimized[6]) <<" ";
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
            cube_marker.lifetime = ros::Duration(0.5); 
          }
           ROS_INFO("FINAL, publish the markers");
           // // Publish the marker
           conefitting_pub_.publish(cube_marker);
          

          // ---[ Convert clusters into the PointCloud message
          //std::vector<sensor_msgs::PointCloud> clusters_cone_inliers;
          //getClustersFromPointCloud2<Point> (*cloud_cluster_ptr, clusters2, clusters_cone_inliers); 
          //ROS_INFO(" clusters.size() %d",(int)cluster.size());
          
          //publishClusterMarkers(*cloud_ptr,  converted_cloud.header);

           
           response.I=I;
          }
         
         ROS_INFO_STREAM("In total, cone fitting took " << ros::Time::now() - start_time << " seconds");
         return true;
      
        }
      }   
   }     










} //namespace visual_perception

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "cone_fitting_node");
  ros::NodeHandle nh;

  visual_perception::ConeFitter node(nh);

  ros::spin();
  return 0;
}
