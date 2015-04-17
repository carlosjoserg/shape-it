
/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): Matei Ciocarlie

#include <ros/ros.h>

#include <string>
#include <vector>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

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

#include "visual_perception/TabletopSegmentation.h"
#include "visual_perception/CylinderFitting.h"
#include "visual_perception/Cylinder.h"
/*! Simply pings the cylinder fitting service and prints out the result.*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_cylinder_fitting");
  ros::NodeHandle nh;

  std::string service_name("cylinder_fitting_srv");
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);

  std::string topic = nh.resolveName("cloud_pcd");
  ROS_INFO("Cylinder fitting service called; waiting for a point_cloud2 on topic %s", topic.c_str());
  
  sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh, ros::Duration(3.0));
  
  visual_perception::CylinderFitting fitting_srv;

  pcl::copyPointCloud(*recent_cloud, fitting_srv.request.cluster);
  ROS_INFO("Cylinder fitting service  waiting for service on topic fitting_srv");
  if (!ros::service::call(service_name, fitting_srv))
  {
    ROS_ERROR("Call to cylinder fitting service failed");
    exit(0);
  }
  else 
  {
    visual_perception::Cylinder cylinder;
    cylinder = fitting_srv.response.cylinder;
    ROS_INFO("Raggio del cilindro: %f, Altezza del cilindro  %f", cylinder.r, cylinder.h);
    ROS_INFO("Cylinder fitting DONE !!!");
  }
 return true;
};
