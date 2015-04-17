// Author(s): Luca Gemma

#include <string>

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cone.h>
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


#include "grasp_planner/Trajectory.h"
#include "grasp_planner/GraspPlanning.h"
#include "grasp_planner/TrajectoryDemoOnline.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>



namespace visual_perception {

class ObjectManager 
{
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;
  
private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher objectmanaging_pub_;
  //! Service server for object detection
  ros::ServiceServer object_managing_srv_;

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


  bool serviceCallback(ObjectManaging::Request &request, ObjectManaging::Response &response);
  

  

 public:
 //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  ObjectManager(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
   num_markers_published_ = 1;
   current_marker_id_ = 1;

   objectmanaging_pub_= nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

   object_managing_srv_ = nh_.advertiseService(nh_.resolveName("object_managing_srv"),    &ObjectManager::serviceCallback, this);

    

  }

  //! Empty stub
  ~ObjectManager() {}

};



/*! call serviceCallback
 */
bool ObjectManager::serviceCallback(ObjectManaging::Request &request, ObjectManaging::Response &response)
{  
  int n_obj;
  n_obj= request.init;
  int int_traj =(int)4;
  moveit_msgs::RobotTrajectory trajectory;
  
  int cyl = 0;
  int sph = 0;
  int con = 0;   
  int plan = 0;

  float I_cyl[10];
  float I_sph[10];
  float I_con[10];
  float I_plan[10];

  float I_cyl_best_fit = (float)0;
  float I_sph_best_fit = (float)0;
  float I_con_best_fit = (float)0;
  float I_plan_best_fit= (float)0;
  
  visual_perception::Cylinder cylinder_resp[10];
  visual_perception::Sphere sphere_resp[10];
  visual_perception::Cone cone_resp[10];
  visual_perception::Plane plane_resp[10];
  visual_perception::Cylinder cylinder_best_fit;
  visual_perception::Sphere sphere_best_fit;
  visual_perception::Cone cone_best_fit;
  visual_perception::Plane plane_best_fit;

  

  if (n_obj ==0)
  {  
   
   
   ros::NodeHandle nh;
   std::string service_name("/tabletop_segmentation");
   while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
    {
     ROS_INFO("Waiting for service %s...", service_name.c_str());
    }
    if (!nh.ok()) exit(0);

    visual_perception::TabletopSegmentation segmentation_srv;
    if (!ros::service::call(service_name, segmentation_srv))
     {
       ROS_ERROR("Call to segmentation service failed");
       exit(0);
      }
     if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
     {
       ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
       exit(0);
     }
     ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());
     if (segmentation_srv.response.clusters.empty()) exit(0);

     //also test segmenting with input table
     //sleep(0.5);
     // segmentation_srv.request.table = segmentation_srv.response.table;
     // ROS_INFO("Re-segmenting with the same table shifted 10 cm closer to the robot");
     // for(size_t i=0; i<segmentation_srv.request.table.convex_hull.vertices.size(); i++)
     // {
     //   segmentation_srv.request.table.convex_hull.vertices[i].x -= .10;
     // }
     // if (!ros::service::call(service_name, segmentation_srv))
     // {
     //   ROS_ERROR("Call to segmentation service failed");
     //   exit(0);
     // }
     // if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
     // {
     //   ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
     //   exit(0);
     // }
     ROS_INFO("Segmentation service with table input succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size()); 

     visual_perception::Table table;
     table=segmentation_srv.response.table;
     response.table=table;
        
     float  a = segmentation_srv.response.table.a;
     float  b = segmentation_srv.response.table.b;
     float  c = segmentation_srv.response.table.c;
     float  d = segmentation_srv.response.table.d;

     ROS_INFO("Table acquired.");


     std::string service_name2("cylinder_fitting_srv");
     while ( !ros::service::waitForService(service_name2, ros::Duration().fromSec(3.0)) && nh.ok() )
     {
      ROS_INFO("Waiting for service %s...", service_name2.c_str());
     }
     if (!nh.ok()) exit(0);

     std::string service_name3("sphere_fitting_srv");
     while ( !ros::service::waitForService(service_name3, ros::Duration().fromSec(3.0)) && nh.ok() )
     {
       ROS_INFO("Waiting for service %s...", service_name3.c_str());
     }
     if (!nh.ok()) exit(0);

     std::string service_name4("cone_fitting_srv");
     while ( !ros::service::waitForService(service_name4, ros::Duration().fromSec(3.0)) && nh.ok() )
     {
       ROS_INFO("Waiting for service %s...", service_name4.c_str());
     }
     if (!nh.ok()) exit(0);


     std::string service_name5("parallelepiped_fitting_srv");
     while ( !ros::service::waitForService(service_name5, ros::Duration().fromSec(3.0)) && nh.ok() )
     {
       ROS_INFO("Waiting for service %s...", service_name5.c_str());
     }
     if (!nh.ok()) exit(0);


     // std::string topic = nh.resolveName("cloud_pcd");
     // ROS_INFO("Cylinder fitting service called; waiting for a point_cloud2 on topic %s", topic.c_str());
  
     // sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh, ros::Duration(3.0));
     std::vector<sensor_msgs::PointCloud> clusters; 
     //sensor_msgs::PointCloud2::ConstPtr clusters = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(service_name, ros::Duration(3.0));


     clusters= segmentation_srv.response.clusters;

     ROS_INFO("Clusters acquired.");


  
     //  fitting_srv.request.cluster2= segmentation_srv.response.cluster;
     int size;
     size =clusters.size();
     response.n_object=size;
     ROS_INFO("%d  Object detetected "  , size) ;




     for (size_t i = 0; i < size; ++i)
     { 
       visual_perception::CylinderFitting cylinder_fitting_srv;

       cylinder_fitting_srv.request.cluster=clusters;

       cylinder_fitting_srv.request.n_object=i;

       ROS_INFO("Cylinder fitting service  waiting for service on topic cylinder_fitting_srv");
       if (!ros::service::call(service_name2, cylinder_fitting_srv))
       {
         ROS_ERROR("Call to cylinder fitting service failed");
         exit(0);
        }
        else 
        { 
          if (cylinder_fitting_srv.response.result != cylinder_fitting_srv.response.SUCCESS)
          {
            ROS_ERROR("CylinderFitting service returned error %d", cylinder_fitting_srv.response.result);
            exit(0);
           }
          ROS_INFO("Call to cylinder fitting DONE !!!");

          visual_perception::Cylinder cylinder;    
          cylinder = cylinder_fitting_srv.response.cylinder;
          ROS_INFO("Raggio del cilindro: %f, Altezza del cilindro  %f", (float)cylinder.r, (float)cylinder.h);
          ROS_INFO("Cylinder fitting DONE !!!");
          ROS_INFO("Indice di qualita del fitting %f",(float)cylinder_fitting_srv.response.I);
          if((float)cylinder_fitting_srv.response.I>(float)0.7)
          {
           cylinder_resp[cyl]=cylinder;
           I_cyl[cyl] = (float)cylinder_fitting_srv.response.I;
           if((int)cyl==(int)0) cylinder_best_fit=cylinder_resp[cyl]; I_cyl_best_fit=(float)cylinder_fitting_srv.response.I;
           if((int)cyl>(int)0) 
           {
            if(I_cyl[cyl]>I_cyl[cyl-1]) cylinder_best_fit=cylinder_resp[cyl]; I_cyl_best_fit=(float)cylinder_fitting_srv.response.I;
           }
           cyl = cyl+1;
          }
        }  

        //float dist_cyl_cent =  pcl::pointToPlaneDistanceSigned (cylinder_fitting_srv.response.cylinder.pose.pose.position,  a,  b,  c,  d);
        //sleep(1);

       visual_perception::SphereFitting sphere_fitting_srv;

       sphere_fitting_srv.request.cluster=clusters;

       sphere_fitting_srv.request.n_object=i;

         
 
       ROS_INFO("sphere fitting service  waiting for service on topic sphere_fitting_srv");
       if (!ros::service::call(service_name3, sphere_fitting_srv))
       {
         ROS_ERROR("Call to sphere fitting service failed");
         exit(0);
        }
       else 
       {      
         if (sphere_fitting_srv.response.result != sphere_fitting_srv.response.SUCCESS)
         {
           ROS_ERROR("Segmentation service returned error %d", sphere_fitting_srv.response.result);
           exit(0);
          }

         ROS_INFO("Call to sphere fitting DONE !!!");

         visual_perception::Sphere sphere;    
         sphere = sphere_fitting_srv.response.sphere;
         ROS_INFO("Raggio della sphere: %f", (float)sphere.r);
         ROS_INFO("sphere fitting DONE !!!");
         ROS_INFO("Indice di qualita del fitting %f",(float)sphere_fitting_srv.response.I);

          if((float)sphere_fitting_srv.response.I>(float)0.7)
          {
           sphere_resp[sph]=sphere;
           I_sph[sph] = (float)sphere_fitting_srv.response.I;
           if((int)sph==(int)0) sphere_best_fit=sphere_resp[sph]; I_sph_best_fit=(float)sphere_fitting_srv.response.I;
           if((int)sph>(int)0) 
           {
            if(I_sph[sph]>I_sph[sph-1]) sphere_best_fit=sphere_resp[sph]; I_sph_best_fit=(float)sphere_fitting_srv.response.I;
           }
           sph = sph+1;
          } 


        }
        //float dist_sph_cent = pcl::pointToPlaneDistanceSigned (sphere_fitting_srv.response.sphere.pose.pose.position,  a,  b,  c,  d);
        sleep(1);
        visual_perception::ConeFitting cone_fitting_srv;
        cone_fitting_srv.request.cluster=clusters;

        cone_fitting_srv.request.n_object=i;  

       ROS_INFO("cone fitting service  waiting for service on topic cone_fitting_srv");
       if (!ros::service::call(service_name4, cone_fitting_srv))
       {
         ROS_ERROR("Call to cone fitting service failed");
         exit(0);
        }
       else 
       {    
         if (cone_fitting_srv.response.result != cone_fitting_srv.response.SUCCESS)
         {
           ROS_ERROR("Segmentation service returned error %d", cone_fitting_srv.response.result);
           exit(0);
          }

         ROS_INFO("Call to cone fitting DONE !!!");

 
         visual_perception::Cone cone;    
         cone = cone_fitting_srv.response.cone;
         ROS_INFO("Altezza del cono %f  e angolo di apertura %f", (float)cone.h ,(float)cone.angle);
         ROS_INFO("Cone fitting DONE !!!");
         ROS_INFO("Indice di qualita del fitting %f",(float)cone_fitting_srv.response.I);

         if((float)cone_fitting_srv.response.I>(float)0.7)
          {
           cone_resp[con]=cone;
            I_con[con] = (float)cone_fitting_srv.response.I;
           if((int)con==(int)0) cone_best_fit=cone_resp[con]; I_con_best_fit=(float)cone_fitting_srv.response.I;
           if((int)sph>(int)0) 
           {
            if(I_con[con]>I_con[con-1]) cone_best_fit=cone_resp[con]; I_con_best_fit=(float)cone_fitting_srv.response.I;
           }
           
           con = con+1;
          } 





        }
       
       //float dist_con_cent = pcl::pointToPlaneDistanceSigned (cone_fitting_srv.response.cone.pose.pose.position,  a,  b,  c,  d);
       sleep(1);
       visual_perception::ParallelepipedFitting parallelepiped_fitting_srv;
       parallelepiped_fitting_srv.request.cluster=clusters;

       parallelepiped_fitting_srv.request.n_object=i;  

       parallelepiped_fitting_srv.request.table=table; 

       ROS_INFO("parallelepiped fitting service  waiting for service on topic parallelepiped_fitting_srv");
       if (!ros::service::call(service_name5, parallelepiped_fitting_srv))
       {
        ROS_ERROR("Call to cone fitting service failed");
          exit(0);
        }
         else 
        {    
          if (parallelepiped_fitting_srv.response.result != parallelepiped_fitting_srv.response.SUCCESS)
          {
           ROS_ERROR("Segmentation service returned error %d", parallelepiped_fitting_srv.response.result);
           exit(0);
          }

          ROS_INFO("Call to cone fitting DONE !!!");

 
       visual_perception::Plane plane1;    
       plane1 = parallelepiped_fitting_srv.response.plane_1;
       ROS_INFO("parallelepiped fitting service finds the first plane");

       ROS_INFO("Indice di qualita del fitting %f",(float)parallelepiped_fitting_srv.response.I);
        
       if((float)parallelepiped_fitting_srv.response.I>(float)0.7)
        {
          plane_resp[plan]=plane1;
          I_plan[plan] = (float)parallelepiped_fitting_srv.response.I;
          if((int)plan==(int)0) plane_best_fit=plane_resp[plan]; I_plan_best_fit=(float)parallelepiped_fitting_srv.response.I;
          if((int)plan>(int)0) 
          {
           if(I_plan[plan]>I_plan[plan-1]) plane_best_fit=plane_resp[sph]; I_plan_best_fit=(float)parallelepiped_fitting_srv.response.I;
           }
          plan = plan+1;
        } 
      }
    }
    



     /*! call  GraspPlanning
 */


     ROS_INFO("Start grasp planning for the best fitted object");

     tf::StampedTransform world_link_2_wrist_stamped;
     ros::Time now = ros::Time::now();
            
        // listener_.waitForTransform("world_link", "left_arm_7_link", now, ros::Duration(4));
        // listener_.lookupTransform("world_link", "left_arm_7_link", now, world_link_2_wrist_stamped);
        // tf::Transform world_link_2_wrist;
        // world_link_2_wrist.setOrigin(world_link_2_wrist_stamped.getOrigin());
        // world_link_2_wrist.setBasis(world_link_2_wrist_stamped.getBasis());
        // tf::Vector3 origin_wrist = world_link_2_wrist.getOrigin();
        // tf::Quaternion quaternion_wrist = world_link_2_wrist.getRotation();

        // tf::Transform wrist_pose;

        // wrist_pose.setOrigin(origin_wrist);
        // wrist_pose.setRotation(quaternion_wrist);
        // geometry_msgs::Pose wrist_pose_msg;
        // tf::poseTFToMsg(wrist_pose,wrist_pose_msg);
        //tf::poseStampedTFToMsg(wrist_pose_stamped,wrist_pose);

      std::string service_name6("trajectory_manager_online_srv");
         while ( !ros::service::waitForService(service_name6, ros::Duration().fromSec(3.0)) && nh.ok() )
        {
         ROS_INFO("Waiting for service %s...", service_name6.c_str());
        }
        if (!nh.ok()) exit(0);

         grasp_planner::TrajectoryDemoOnline trajectory_manager_online_srv;

    if((float)I_cyl_best_fit> (float)I_sph_best_fit & (float)I_cyl_best_fit > (float)I_con_best_fit & (float)I_cyl_best_fit> (float)I_plan_best_fit)
       {
      
         ROS_INFO("Start grasp planning for the cylinder");

        
       
      
        trajectory_manager_online_srv.request.ob=(int)1;  

        trajectory_manager_online_srv.request.cylinder=cylinder_best_fit;



        //grasp_planning_srv.request.WristPose.pose= wrist_pose_msg;



        ROS_INFO("trajectory_manager_online_srv service  waiting for service on topic trajectory_manager_online_srv");
        if (!ros::service::call(service_name6, trajectory_manager_online_srv))
        {
         ROS_ERROR("Call to grasp planning service failed");
         //exit(0);
        }
        else 
        {    
         if (trajectory_manager_online_srv.response.result != trajectory_manager_online_srv.response.SUCCESS)
          {
           ROS_ERROR("trajectory_manager_online_srv service returned error %d", trajectory_manager_online_srv.response.result);
           //exit(0);
          }
          else
          {  
            ROS_INFO("Call to trajectory_manager_online_srv srv fitting DONE !!!");
            grasp_planner::Trajectory traj_cyl[int_traj];
            for(int i =0;i<int_traj;i++)
            {
              traj_cyl[i] = trajectory_manager_online_srv.response.traj[i];
            }
            trajectory = trajectory_manager_online_srv.response.moveit_trajectory;         
            ROS_INFO("GraspPlanning find the Trajectory to the cylinder");
            }
          }
        }
      // else
      // {
      if(I_sph_best_fit> I_cyl_best_fit & I_sph_best_fit > I_con_best_fit & I_sph_best_fit> I_plan_best_fit)
      {
      
        trajectory_manager_online_srv.request.ob=2;  

        trajectory_manager_online_srv.request.sphere=sphere_best_fit; 

        ROS_INFO("trajectory_manager_online_srv  waiting for service on topic trajectory_manager_online_srv");
        if (!ros::service::call(service_name6, trajectory_manager_online_srv))
        {
         ROS_ERROR("Call to grasp planning service failed");
         //exit(0);
        }
        else 
        {    
         if (trajectory_manager_online_srv.response.result != trajectory_manager_online_srv.response.SUCCESS)
          {
           ROS_ERROR("grasp planning service returned error %d", trajectory_manager_online_srv.response.result);
           //exit(0);
          }
          else
          {  
            ROS_INFO("Call to trajectory_manager_online_srv fitting DONE !!!");
            grasp_planner::Trajectory traj_sph[int_traj];
            for(int i =0;i<int_traj;i++)
           {
             traj_sph[i] = trajectory_manager_online_srv.response.traj[i];
           }
           trajectory = trajectory_manager_online_srv.response.moveit_trajectory;          
           ROS_INFO("GraspPlanning find the Trajectory to the sphere");
          }
        }
      }  
      // else
      // {  
      if(I_con_best_fit> I_cyl_best_fit & I_con_best_fit > I_sph_best_fit & I_con_best_fit> I_plan_best_fit)
      {
      
         ROS_INFO("Start grasp planning for the cone");
      
        trajectory_manager_online_srv.request.ob=3;  

        trajectory_manager_online_srv.request.cone=cone_best_fit; 

        ROS_INFO("trajectory_manager_online_srv  waiting for service on trajectory_manager_online_srv");
        if (!ros::service::call(service_name6, trajectory_manager_online_srv))
        {
         ROS_ERROR("Call to trajectory_manager_online_srv failed");
         //exit(0);
        }
        else 
        {    
          if (trajectory_manager_online_srv.response.result != trajectory_manager_online_srv.response.SUCCESS)
          {
           ROS_ERROR("trajectory_manager_online_srv returned error %d", trajectory_manager_online_srv.response.result);
           //exit(0);
          }
          else
          {  
           ROS_INFO("Call to trajectory_manager_online_srv fitting DONE !!!");
           grasp_planner::Trajectory traj_cone[int_traj];
           for(int i =0;i<int_traj;i++)
           {
             traj_cone[i] = trajectory_manager_online_srv.response.traj[i];
           }  
           trajectory = trajectory_manager_online_srv.response.moveit_trajectory;         
           ROS_INFO("trajectory_manager_online_srv find the Trajectory to the cone");
          }
        }  
      } 
       // else
       // {
       if(I_plan_best_fit> I_cyl_best_fit & I_plan_best_fit > I_sph_best_fit & I_plan_best_fit> I_con_best_fit)
       {
      
         trajectory_manager_online_srv.request.ob=4;  

         trajectory_manager_online_srv.request.plane=plane_best_fit; 

         ROS_INFO("trajectory_manager_online_srv  waiting for service on topic trajectory_manager_online_srv");
         if (!ros::service::call(service_name6, trajectory_manager_online_srv))
         {
          ROS_ERROR("Call to trajectory_manager_online_srv service failed");
          //exit(0);
         }
        else 
        {    
          if (trajectory_manager_online_srv.response.result != trajectory_manager_online_srv.response.SUCCESS)
          {
           ROS_ERROR("grasp planning service returned error %d", trajectory_manager_online_srv.response.result);
           //exit(0);
          }
          else
          { 
           ROS_INFO("Call to trajectory_manager_online_srv fitting DONE !!!");
           grasp_planner::Trajectory traj_plane[int_traj];
           for(int i =0;i<int_traj;i++)
           {
             traj_plane[i] = trajectory_manager_online_srv.response.traj[i];
           } 

           trajectory = trajectory_manager_online_srv.response.moveit_trajectory;         
           ROS_INFO("trajectory_manager_online_srv find the Trajectory to the plane");

          }
      // }
      // }
      // }
        }
      }
         ROS_INFO("Start sending response");
 

       //response.cylinder[cyl-1] = cylinder_resp[cyl-1];
       // Hard code to send response
        for(int i =0;i<cyl;i++)
        {
        response.cylinder[i] = cylinder_resp[i];
        }
        response.cyl = cyl;
       //  if (cyl==0)
       // {
       //  for(int i =0;i<10;i++)
       //  {
       //   response.cylinder[i] = cylinder_resp[i];
       //  }
       // }


        for(int i =0;i<sph;i++)
        {
         response.sphere[i] = sphere_resp[i];
        }
        response.sph = sph;
       // if (sph==0)
       // {
       //  for(int i =0;i<10;i++)
       //  {
       //   response.sphere[i] = sphere_resp[i];
       //  }
       // } 



        for(int i =0;i<con;i++)
        {
        response.cone[i] = cone_resp[i];
        }
        response.con = con;
       //  if (con==0)
       // {
       //  for(int i =0;i<10;i++)
       //  {
       //   response.cone[i] = cone_resp[i];
       //  }
       // }
       for(int i =0;i<plan;i++)
        {
        response.plane[i] = plane_resp[i];
        }
        response.plan = plan;
        
        sleep(1);

       
    
  }
} 


}//namespace visual_perception













int main(int argc, char **argv) 
{
  ros::init(argc, argv, "object_managing_node");
  ros::NodeHandle nh;

  visual_perception::ObjectManager node(nh);

  ros::spin();
  return 0;
};









































