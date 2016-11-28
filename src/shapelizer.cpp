// ROS STUFF

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>

// CGAL STUFF

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/jet_smooth_point_set.h>

#include <iostream>
#include <fstream>
#include <cstdlib>

// Type declarations
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::FT                                           FT;
typedef Kernel::Point_3                                      Point;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;

// In Efficient_RANSAC_traits the basic types, i.e., Point and Vector types
// as well as iterator type and property maps, are defined.
typedef CGAL::Shape_detection_3::Efficient_RANSAC_traits<Kernel,
  Pwn_vector, Point_map, Normal_map>            Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection_3::Cone<Traits>             Cone;
typedef CGAL::Shape_detection_3::Cylinder<Traits>         Cylinder;
typedef CGAL::Shape_detection_3::Plane<Traits>            Plane;
typedef CGAL::Shape_detection_3::Sphere<Traits>           Sphere;
typedef CGAL::Shape_detection_3::Torus<Traits>            Torus;

// std::string filename = "02018R3M";
std::string filename = "snapshot";
// std::string filename = "cylinder";

// ROS
ros::Publisher marker_pub;
ros::Publisher segmented_cloud_pub;

// CGAL

Efficient_ransac ransac;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

void ShapelizerCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud)
{

  

  
  Pwn_vector points;
  
  for(auto point : *_cloud)
  {
    Kernel::Point_3 single_point(point.x, point.y, point.z);
    Point_with_normal single_pwn;
    single_pwn.first = single_point;
    points.push_back(single_pwn);
  }


    
  /*const unsigned int nb_neighb = 24; // default is 24 for real-life point sets
  CGAL::jet_smooth_point_set(points.begin(), points.end(), CGAL::First_of_pair_property_map<Point_with_normal>(), nb_neighb);*/
  
  // Normal estimation using CGAL
  const int nb_neighbors = 24; // K-nearest neighbors = 3 rings
  Point_map point_map;
  Normal_map normal_map;
  CGAL::pca_estimate_normals(points.begin(), points.end(),
					      CGAL::First_of_pair_property_map<Point_with_normal>(),
					      CGAL::Second_of_pair_property_map<Point_with_normal>(),
					      nb_neighbors);
  

  
  // Orients normals.
  // Note: mst_orient_normals() requires an iterator over points
  // as well as property maps to access each point's position and normal.
  CGAL::mst_orient_normals(points.begin(), points.end(),
				CGAL::First_of_pair_property_map<Point_with_normal>(),
				CGAL::Second_of_pair_property_map<Point_with_normal>(),
				nb_neighbors);
  
  // Algorithm parameters
  /*int k = 120;                 // size of neighborhood. The bigger the smoother the result will be.
                               // This value should bigger than 1.
  double sharpness_angle = 25; // control sharpness of the result.
                               // The bigger the smoother the result will be
  int iter_number = 2;         // number of times the projection is applied
  
  for (int i = 0; i < iter_number; ++i)
  {
    CGAL::bilateral_smooth_point_set<Concurrency_tag>(
          points.begin(), 
          points.end(),
          CGAL::First_of_pair_property_map<Point_with_normal>(),
          CGAL::Second_of_pair_property_map<Point_with_normal>(),
          k,
          sharpness_angle);
  }*/

  // Provides the input data.
  ransac.set_input(points);
  // Sets parameters for shape detection.
  Efficient_ransac::Parameters parameters;

  // Sets probability to miss the largest primitive at each iteration.
  parameters.probability = 0.9;

  // Detect shapes with at least 500 points.
  parameters.min_points = 10;

 
  parameters.epsilon = 0.003;
  // Sets maximum normal deviation.
  // 0.9 < dot(surface_normal, point_normal);
  parameters.normal_threshold = 0.8;

  // cluster_epsilon: Clustering of the points into connected components covered by a 
  // detected shape is controlled via parameter cluster_epsilon. For developable shapes that 
  // admit a trivial planar parameterization (plane, cylinder, cone) the points covered by a shape are 
  // mapped to a 2D parameter space chosen to minimize distortion and best preserve arc-length distances. 
  // This 2D parameter space is discretized using a regular grid, and a connected component search is 
  // performed to identify the largest cluster. Parameter cluster_epsilon defines the spacing between 
  // two cells of the regular grid, so that two points separated by a distance of at most 22‾‾√, 
  // cluster_epsilon are considered adjacent. For non-developable shapes the connected components are 
  // identified by computing a neighboring graph in 3D and walking in the graph. 
  parameters.cluster_epsilon = 0.02;
  
  // Detects shapes
  ransac.detect(parameters);
  // Prints number of detected shapes and unassigned points.
   std::cout << ransac.shapes().end() - ransac.shapes().begin() << " detected shapes, "
     << ransac.number_of_unassigned_points()
     << " unassigned points." << std::endl;

  // Efficient_ransac::shapes() provides
  // an iterator range to the detected shapes.
  Efficient_ransac::Shape_range shapes = ransac.shapes();
  Efficient_ransac::Shape_range::iterator it = shapes.begin();
  
  int id_cyl = 0;
  int id_sph = 0;
  // Container for all markers
  visualization_msgs::MarkerArray markers;
  
  pcl::PointCloud<pcl::PointXYZRGB> segmented_cloud;
  segmented_cloud.width = points.size();
  segmented_cloud.height = 1;
  segmented_cloud.is_dense = true;
  
  segmented_cloud.header = _cloud->header;
  
  while (it != shapes.end()) {

    // Get specific parameters depending on detected shape.
    if (Plane* plane = dynamic_cast<Plane*>(it->get()))
      {
        Kernel::Vector_3 normal = plane->plane_normal();
        std::cout << "Plane with normal " << normal
                << std::endl;

        // Plane shape can also be converted to Kernel::Plane_3
        // std::cout << "Kernel::Plane_3: " << static_cast<Kernel::Plane_3>(*plane) << std::endl;
      }
    else if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get()))
      {
        Kernel::Line_3 axis = cyl->axis();
        FT radius = cyl->radius();
        std::cout << "Cylinder with axis " << axis
                  << " and radius " << radius
                  << std::endl;
		  
        // Build R
        tf::Vector3 w(axis.direction().dx(), axis.direction().dy(), axis.direction().dz());
	tf::Vector3 u(1, 0, 0);
	if ( fabs(w.dot(u)) > 1.0 - 1.0e-4)
	{
	    u = tf::Vector3(0, 1, 0);
	}
	tf::Vector3 v = w.cross(u).normalized();
	u = v.cross(w).normalized();
	tf::Matrix3x3 rotation;
	rotation[0] = u;
	rotation[1] = v;
	rotation[2] = w;
	rotation = rotation.transpose();
	tf::Quaternion orientation;
	rotation.getRotation(orientation);
	
	// compute the real center and height
	// get the current axis position
	tf::Vector3 p0(axis.point().x(), axis.point().y(), axis.point().z());
	// generic point in the cluster to be read from the cloud
	tf::Vector3 p;
	// real center of the cylinder
	tf::Vector3 p0_corrected;
	// all projection values of points in axis
	std::vector<float> cyl_lambdas;

	// project all points in the cluster onto the cylinder axis
	// Iterates through point indices assigned to each detected shape.
	std::vector<std::size_t>::const_iterator index_it = (*it)->indices_of_assigned_points().begin();
	while (index_it != (*it)->indices_of_assigned_points().end())
	{  
	  // Retrieves point
	  const Point_with_normal &pp = *(points.begin() + (*index_it));
	  p[0] = pp.first.x();
	  p[1] = pp.first.y();
	  p[2] = pp.first.z();
	  cyl_lambdas.push_back( w.dot(p-p0) );
	  
	  pcl::PointXYZRGB pcl_point;
	  pcl_point.x = pp.first.x();
	  pcl_point.y = pp.first.y();
	  pcl_point.z = pp.first.z();
	  double color = (double) std::rand()/(RAND_MAX + 1.0);
	  pcl_point.r = color;
	  pcl_point.g = 1 - color;
	  pcl_point.b = color;
	  segmented_cloud.push_back(pcl_point);
	  // Proceeds with next point.
	  index_it++;
	}
	float lambda_max = *std::max_element(cyl_lambdas.begin(), cyl_lambdas.end());
	float lambda_min = *std::min_element(cyl_lambdas.begin(), cyl_lambdas.end());
	float lambda_mean = 0.5*(lambda_max + lambda_min);
	float height = lambda_max - lambda_min;
	p0_corrected = p0 + lambda_mean*w;
	
	if( (fabs(radius) > 0.001) && (fabs(radius) < 0.010 ) && (height < 0.08) )
	{
	  // create the marker
	  visualization_msgs::Marker cyl_marker;
	  cyl_marker.type = visualization_msgs::Marker::CYLINDER;;
	  cyl_marker.header.frame_id = "map";

	  cyl_marker.ns = "cylinders";
	  cyl_marker.id = id_cyl;
	  id_cyl++;
	  cyl_marker.action = visualization_msgs::Marker::ADD;
	  cyl_marker.lifetime = ros::Duration();

	  cyl_marker.pose.position.x = p0_corrected[0];
	  cyl_marker.pose.position.y = p0_corrected[1];
	  cyl_marker.pose.position.z = p0_corrected[2];
	  cyl_marker.pose.orientation.x = orientation[0];
	  cyl_marker.pose.orientation.y = orientation[1];
	  cyl_marker.pose.orientation.z = orientation[2];
	  cyl_marker.pose.orientation.w = orientation[3];

	  cyl_marker.color.r = 1.0f;
	  cyl_marker.color.g = 1.0f;
	  cyl_marker.color.b = 0.0f;
	  cyl_marker.color.a = 1.0f;

	  cyl_marker.scale.x = 2*fabs(radius);
	  cyl_marker.scale.y = 2*fabs(radius);
	  cyl_marker.scale.z = height;
	  
	  markers.markers.push_back(cyl_marker);
	}
      }
    else if (Sphere* sph = dynamic_cast<Sphere*>(it->get()))
      {
	std::cout << "Sphere with center ";
	    Kernel::Point_3 center = sph->center();
	    FT radius = cyl->radius();
	    std::cout << center
			      << " and radius " << radius
			      << std::endl;
	    // create the marker
	visualization_msgs::Marker sph_marker;
	sph_marker.type = visualization_msgs::Marker::SPHERE;;
	sph_marker.header.frame_id = "map";

	sph_marker.ns = "spheres";
	sph_marker.id = id_sph;
	id_sph++;
	sph_marker.action = visualization_msgs::Marker::ADD;
	sph_marker.lifetime = ros::Duration();

	sph_marker.pose.position.x = center.x();
	sph_marker.pose.position.y = center.y();
	sph_marker.pose.position.z = center.z();
	sph_marker.pose.orientation.w = 1;

	sph_marker.color.r = 1.0f;
	sph_marker.color.g = 0.0f;
	sph_marker.color.b = 1.0f;
	sph_marker.color.a = 0.3f;

	sph_marker.scale.x = radius;
	sph_marker.scale.y = radius;
	sph_marker.scale.z = radius;
	
	markers.markers.push_back(sph_marker);
      }
    else
      {
        // Prints the parameters of the detected shape.
        // This function is available for any type of shape.
        std::cout << (*it)->info() << std::endl;
      }

    // Proceeds with next detected shape.
    it++;
  }
  
  if( !markers.markers.empty() )
      marker_pub.publish(markers);
  
  if( !segmented_cloud.empty() )
      segmented_cloud_pub.publish(segmented_cloud);

}

int main(int argc, char** argv)
{
  // PUBLISHER
  ros::init(argc, argv, "shapelizer");
  ros::NodeHandle n;
  
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("shapelization_result", 1, true);
  segmented_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >("segmented_cloud", 1, true);
  
  ros::Subscriber sub_cloud = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >("filtered_cloud", 1, ShapelizerCallback);

  // Marker for the snapshor
  /*visualization_msgs::Marker mesh_marker;

  mesh_marker.action = visualization_msgs::Marker::ADD;
  mesh_marker.ns = std::string("scene");
  mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh_marker.mesh_resource = std::string("package://shape_it/test/") + filename + std::string(".stl");
  mesh_marker.header.frame_id = "map";
  mesh_marker.pose.orientation.w = 1;
  mesh_marker.scale.x = 1.0;
  mesh_marker.scale.y = 1.0;
  mesh_marker.scale.z = 1.0;
  mesh_marker.color.a = 1.0;

  markers.markers.push_back(mesh_marker);
  marker_pub.publish(markers);
  ros::spinOnce();*/

  
  // Points with normals.
 
  // Loads point set from a file.
  // read_xyz_points_and_normals takes an OutputIterator for storing the points
  // and a property map to store the normal vector with each point.
  /*std::ifstream stream(std::string("/home/carlos/Catkin/src/shape-it/test/") + filename + std::string(".xyz") );
  if (!stream ||
    !CGAL::read_xyz_points_and_normals(stream,
      std::back_inserter(points),
      Point_map(),
      Normal_map()))
  {
	std::cerr << "Error: cannot read file 02018R3M.xyz" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << points.size() << " points" << std::endl;*/

  // Instantiates shape detection engine.
  
  // Register shapes for detection
  ransac.add_shape_factory<Plane>();
  // ransac.add_shape_factory<Sphere>();
  ransac.add_shape_factory<Cylinder>();
  // ransac.add_shape_factory<Cone>();
  // ransac.add_shape_factory<Torus>();

  ros::spin();
  
  /*ros::Rate loop(1);
  while( ros::ok() )
  {
    ros::spinOnce();
    rate.sleep();
  }*/
 
  return EXIT_SUCCESS;
}
