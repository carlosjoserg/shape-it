////// CGAL SDF WAY

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
#include <iostream>
#include <fstream>

#include <random>
#include <algorithm>
#include <iterator>
#include <functional>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
int main()
{
	// create and read Polyhedron
	Polyhedron mesh;
	std::ifstream input("/home/carlos/Catkin/src/shape-it/test/02018R3M.off");
	if ( !input || !(input >> mesh) || mesh.empty() )
	{
		std::cerr << "Not a valid off file." << std::endl;
		return EXIT_FAILURE;
	}
	// create a property-map for segment-ids
	typedef std::map<Polyhedron::Facet_const_handle, std::size_t> Facet_int_map;

	Facet_int_map internal_segment_map;

	boost::associative_property_map<Facet_int_map> segment_property_map(internal_segment_map);

	// calculate SDF values and segment the mesh using these parameters.
	double cone_angle = 2.0 / 3.0 * CGAL_PI;
	std::size_t number_of_rays = 25;
	std::size_t number_of_clusters = 10;
	double smoothing_lambda = 0.1;
	bool output_cluster_ids = false;
	//PointPropertyMap ppmap = CGAL::PointPropertyMap();
	//GeomTraits traits = CGAL::GeomTraits();

	std::size_t number_of_segments = CGAL::segmentation_via_sdf_values(mesh, segment_property_map, 
	                                                                   cone_angle,
	                                                                   number_of_rays,
	                                                                   number_of_clusters,
	                                                                   smoothing_lambda,
	                                                                   output_cluster_ids);


	std::cout << "Number of segments: " << number_of_segments << std::endl;


	// print segment-ids
	std::vector<Polyhedron> clusters;
	clusters.resize(number_of_segments);

	// First create an instance of an engine.
    std::random_device rnd_device;
    // Specify the engine and distribution.
    std::mt19937 mersenne_engine(rnd_device());
    std::uniform_int_distribution<int> dist(1, 100);

    auto gen = std::bind(dist, mersenne_engine);
    std::vector<int> blue(number_of_segments);
    std::generate(begin(blue), end(blue), gen);

	for(Polyhedron::Facet_const_iterator facet_it = mesh.facets_begin(); facet_it != mesh.facets_end(); ++facet_it)
	{
		double fraction = (double)segment_property_map[facet_it]/(double)number_of_segments;
		std::cout << 1.0 - fraction << " " << fraction << " " << blue.at(segment_property_map[facet_it])/(double)100.0 << " 1.0 " << std::endl;
	}
	//std::cout << std::endl;

	// for(int i = 0; i < clusters.size(); ++i)
	// {
	// 	std::string file_name = "/home/carlos/Catkin/src/shape-it/test/02018R3M_" + std::to_string(i) + ".off";
	// 	std::ofstream cluster_file;
	// 	cluster_file.open( file_name.c_str() );
	// 	if ( !(cluster_file << clusters.at(i)) || clusters.at(i).empty() )
	// 	{
	// 		std::cerr << "Could not write to off file." << std::endl;
	// 		return EXIT_FAILURE;
	// 	}
	// 	cluster_file.close();
	// }
}



////// PCL SUPERVOXEL WAY
/*
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
									   PointCloudT &adjacent_supervoxel_centers,
									   std::string supervoxel_name,
									   boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);


int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
	pcl::console::print_error ("Syntax is: %s <pcd-file> \n "
								"--NT Dsables the single cloud transform \n"
								"-v <voxel resolution>\n-s <seed resolution>\n"
								"-c <color weight> \n-z <spatial weight> \n"
								"-n <normal_weight>\n", argv[0]);
	return (1);
  }


  PointCloudT::Ptr cloud = boost::shared_ptr <PointCloudT> (new PointCloudT ());
  pcl::console::print_highlight ("Loading point cloud...\n");
  if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud))
  {
	pcl::console::print_error ("Error loading cloud file!\n");
	return (1);
  }


  bool disable_transform = pcl::console::find_switch (argc, argv, "--NT");

  float voxel_resolution = 0.008f;
  bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
  if (voxel_res_specified)
	pcl::console::parse (argc, argv, "-v", voxel_resolution);

  float seed_resolution = 0.1f;
  bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
  if (seed_res_specified)
	pcl::console::parse (argc, argv, "-s", seed_resolution);

  float color_importance = 0.2f;
  if (pcl::console::find_switch (argc, argv, "-c"))
	pcl::console::parse (argc, argv, "-c", color_importance);

  float spatial_importance = 0.4f;
  if (pcl::console::find_switch (argc, argv, "-z"))
	pcl::console::parse (argc, argv, "-z", spatial_importance);

  float normal_importance = 1.0f;
  if (pcl::console::find_switch (argc, argv, "-n"))
	pcl::console::parse (argc, argv, "-n", normal_importance);

  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  if (disable_transform)
	super.setUseSingleCameraTransform (false);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

  PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
  //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
  for ( ; label_itr != supervoxel_adjacency.end (); )
  {
	//First get the label
	uint32_t supervoxel_label = label_itr->first;
	//Now get the supervoxel corresponding to the label
	pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

	//Now we need to iterate through the adjacent supervoxels and make a point cloud of them
	PointCloudT adjacent_supervoxel_centers;
	std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
	for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
	{
	  pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
	  adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
	}
	//Now we make a name for this polygon
	std::stringstream ss;
	ss << "supervoxel_" << supervoxel_label;
	//This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
	addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
	//Move iterator forward to next label
	label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  }

  while (!viewer->wasStopped ())
  {
	viewer->spinOnce (100);
  }
  return (0);
}

void
addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
								  PointCloudT &adjacent_supervoxel_centers,
								  std::string supervoxel_name,
								  boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

  //Iterate through all adjacent points, and add a center point to adjacent point pair
  PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
  for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
  {
	points->InsertNextPoint (supervoxel_center.data);
	points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
	polyLine->GetPointIds ()->SetId (i,i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData,supervoxel_name);
}
*/
