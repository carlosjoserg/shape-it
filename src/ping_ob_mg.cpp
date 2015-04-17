
#include <ros/ros.h>

#include <string>
#include <vector>
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
#include "visual_perception/SphereFitting.h"
#include "visual_perception/ConeFitting.h"
#include "visual_perception/Sphere.h"
#include "visual_perception/Cone.h"
#include "visual_perception/ParallelepipedFitting.h"


/*! Simply pings the tabletop segmentation and recognition services and prints out the result.*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_tabletop_node");
  ros::NodeHandle nh;
  std::string service_name("/tabletop_segmentation");
  //std::string service_name("/tabletop_segmentation");
  //std::string service_name("/segmentation_srv");


  //while(1)
  //{
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
      { if (cylinder_fitting_srv.response.result != cylinder_fitting_srv.response.SUCCESS)
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
      }
       //float dist_cyl_cent =  pcl::pointToPlaneDistanceSigned (cylinder_fitting_srv.response.cylinder.pose.pose.position,  a,  b,  c,  d);
       //sleep(1);
      if((float)cylinder_fitting_srv.response.I<(double)0.83)
      {  
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
      }
      //float dist_sph_cent = pcl::pointToPlaneDistanceSigned (sphere_fitting_srv.response.sphere.pose.pose.position,  a,  b,  c,  d);
      //sleep(1);

      if((float)sphere_fitting_srv.response.I<(double)0.82)
      {
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
      }
       
       float dist_con_cent = pcl::pointToPlaneDistanceSigned (cone_fitting_srv.response.cone.pose.pose.position,  a,  b,  c,  d);
      //sleep(2);
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

       ROS_INFO("Call to parallelepiped fitting DONE !!!");

 
       visual_perception::Plane plane1;    
       plane1 = parallelepiped_fitting_srv.response.plane_1;
       // visual_perception::Plane plane2;    
       // plane2 = parallelepiped_fitting_srv.response.plane_2;
       // visual_perception::Plane plane3;    
       // plane3 = parallelepiped_fitting_srv.response.plane_3;
       visual_perception::Parallelepiped parallelepiped;    
       parallelepiped = parallelepiped_fitting_srv.response.parallelepiped;


       /*ROS_INFO("Altezza del parallelepipedo %f, larghezza %f ,spessore %f", (float)parallelepiped.l ,(float)parallelepiped.b, (float)parallelepiped.w );
       ROS_INFO("parallelepiped fitting DONE !!!");
       ROS_INFO("Indice di qualita del fitting %f",(float)parallelepiped_fitting_srv.response.I);*/
      }
       
      //float dist_par_cent = pcl::pointToPlaneDistanceSigned (parallelepiped_fitting_srv.response.parallelepiped.pose.pose.position,  a,  b,  c,  d);
    }
    }




    }
   //}
 return true;
}