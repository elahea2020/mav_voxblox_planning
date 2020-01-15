#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/merge_integration.h>

#include <voxblox/core/layer.h>

#include <voxblox_ros/conversions.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/mesh_pcl.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>
#include "voxblox_ros/ros_params.h"

#include "voxblox_skeleton/io/skeleton_io.h"
#include "voxblox_skeleton/ros/skeleton_vis.h"
#include "voxblox_skeleton/skeleton_generator.h"

// First iteration 
// just simple subscribers and publishers 
// 
using namespace voxblox;

ros::Publisher skeleton_pub_;
ros::Publisher sparse_graph_pub_;
SkeletonGenerator skeleton_generator_;

float voxel_size = 0.1;
// Creates the skeleton of the entire map (not incrementaly) .
void skeletonize(Layer<EsdfVoxel>* esdf_layer,
                                   voxblox::Pointcloud* pointcloud,
                                   std::vector<float>* distances) {
	const std::string frame_id = "world";
  skeleton_generator_.setEsdfLayer(esdf_layer);

  FloatingPoint min_separation_angle =
      skeleton_generator_.getMinSeparationAngle();
  // nh_private_.param("min_separation_angle", min_separation_angle,
  //                   min_separation_angle);
  skeleton_generator_.setMinSeparationAngle(0.78);
  bool generate_by_layer_neighbors =
      skeleton_generator_.getGenerateByLayerNeighbors();
  // nh_private_.param("generate_by_layer_neighbors", generate_by_layer_neighbors,
  //                   generate_by_layer_neighbors);
  skeleton_generator_.setGenerateByLayerNeighbors(false);

  // int num_neighbors_for_edge = skeleton_generator_.getNumNeighborsForEdge();
  // nh_private_.param("num_neighbors_for_edge", num_neighbors_for_edge,
  //                   num_neighbors_for_edge);
  // skeleton_generator_.setNumNeighborsForEdge(num_neighbors_for_edge);

  FloatingPoint min_gvd_distance = skeleton_generator_.getMinGvdDistance();
  // nh_private_.param("min_gvd_distance", min_gvd_distance, min_gvd_distance);
  skeleton_generator_.setMinGvdDistance(0.7);

	// skeleton_generator_.setVertexPruningRadius()
  skeleton_generator_.generateSkeleton();
  skeleton_generator_.getSkeleton().getEdgePointcloudWithDistances(pointcloud,
                                                                   distances);
  ROS_INFO("Finished generating skeleton.");

  skeleton_generator_.generateSparseGraph();
  ROS_INFO("Finished generating sparse graph.");

  ROS_INFO_STREAM("Total Timings: " << std::endl << timing::Timing::Print());

  // Now visualize the graph.
  const SparseSkeletonGraph& graph = skeleton_generator_.getSparseGraph();
  std::cout << "Size of the graph: |V| = " << graph.getVertexMap().size() << ", |E|"	  		<< graph.getEdgeMap().size() << std::endl; 
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, frame_id, &marker_array);
  sparse_graph_pub_.publish(marker_array);
}

void esdfCallBack(const voxblox_msgs::Layer& layer_msg) {
	// TODO update to get it from launch file
	voxblox::Layer<voxblox::EsdfVoxel>* layer(new Layer<EsdfVoxel>(voxel_size, 16));
	bool success =
      deserializeMsgToLayer<voxblox::EsdfVoxel>(layer_msg, layer);
  std::cout << "succeded in deserilaizing : " << success << std::endl;
  voxblox::Pointcloud pointcloud;
  std::vector<float> distances;
  skeletonize(layer, &pointcloud, &distances);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "skeletonizer_node");
	google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;

	ros::NodeHandle n;
  ros::Subscriber esdf_sub = n.subscribe("/voxblox_node/esdf_map_out", 1, esdfCallBack);
  skeleton_pub_ = n.advertise<pcl::PointCloud<pcl::PointXYZ> >(
        "skeleton", 1, true);
  sparse_graph_pub_ = n.advertise<visualization_msgs::MarkerArray>(
        "sparse_graph", 1, true);
  ros::spin();
	return 0;
}