// Author(s): Luca Gemma

#include <string>

// ROS headers
#include <ros/ros.h>

//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// PCL headers
//#include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/io.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>

// #include <pcl/sample_consensus/sac_model_cylinder.h>
// #include <pcl/sample_consensus/sac_model_sphere.h>
// #include <pcl/sample_consensus/sac_model_cone.h>
// #include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
// #include <pcl/sample_consensus/sac_model_parallel_plane.h>
// #include <pcl/sample_consensus/sac_model_perpendicular_plane.h>


// #include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/sac_model.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/project_inliers.h>
// #include <pcl/surface/convex_hull.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/segmentation/extract_polygonal_prism_data.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl_ros/transforms.h>
// #include <pcl/pcl_base.h>
// #include <pcl/ModelCoefficients.h>

#include "marker_generator.h"
#include "visual_perception/ObjectManaging.h"
#include "visual_perception/TabletopSegmentation.h"
#include "visual_perception/CylinderFitting.h"
#include "visual_perception/Cylinder.h"
#include "visual_perception/SphereFitting.h"
#include "visual_perception/ConeFitting.h"
#include "visual_perception/ParallelepipedFitting.h"
#include "visual_perception/Sphere.h"
#include "visual_perception/Cone.h"





namespace visual_perception {

class PathPlanner 
{
  
  
private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher pathplanning_pub_;
  //! Service server for object detection
  ros::ServiceServer path_planning_srv_;

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  //! A tf transform listener
  tf::TransformListener listener_;

    //! A tf transform broadcaster
  tf::TransformBroadcaster broadcaster_;

  //------------------ Callbacks -------------------

  
  //! Callback for service calls


  bool serviceCallback(PathPlanning::Request &request, PathPlanning::Response &response);
  

  

 public:
 //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  PathPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
   num_markers_published_ = 1;
   current_marker_id_ = 1;

   pathplanning_pub_= nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("path_planning_out"), 10);

   path_planning_srv_ = nh_.advertiseService(nh_.resolveName("object_managing_srv"),    &PathPlanner::serviceCallback, this);

    

  }

  //! Empty stub
  ~PathPlanner() {}

};



/*! Create the path desidered according of shape models.
 */
bool PathPlanner::serviceCallback(PathPlanning::Request &request, PathPlanning::Response &response)
{  

  
geometry_msgs/PoseStamped pose_dobj;

geometry_msgs/PoseStamped pose_wr;

pose_obj=request.pose_object;

pose_wr=request.pose_wrist;

double roll_ob, pitch_ob, yaw_ob;
tf::quaternionMsgToTF(msg->orientation, pose_obj.quaternion);
tf::Matrix3x3(q).getRPY(roll_ob, pitch_ob, yaw_ob);
ROS_INFO("RPY = (%lf, %lf, %lf)", roll_ob, pitch_ob, yaw_ob);

double roll_wr, pitch_wr, yaw_wr;
tf::quaternionMsgToTF(msg->orientation, pose_wr.quaternion);
tf::Matrix3x3(q).getRPY(roll_wr, pitch_wr, yaw_wr);
ROS_INFO("RPY = (%lf, %lf, %lf)", roll_wr, pitch_wr, yaw_wr);




} //namespace visual_perception

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "path_planning_node");
  ros::NodeHandle nh;

  visual_perception::PathPlanner node(nh);

  ros::spin();
  return 0;
}
