#ifndef RRT_PLANNER_VOXBLOX_OMPL_RRT_VOXBLOX_H_
#define RRT_PLANNER_VOXBLOX_OMPL_RRT_VOXBLOX_H_

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include <ros/ros.h>

#include "rrt_planner_voxblox/ompl/mav_setup.h"

namespace mav_planning {

class OmplRrtVoxblox {
 public:
  OmplRrtVoxblox(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~OmplRrtVoxblox() {}

  inline void setRobotRadius(double robot_radius) {
    robot_radius_ = robot_radius;
  }
  void setBounds(const Eigen::Vector3d& lower_bound,
                 const Eigen::Vector3d& upper_bound);

  // Both are expected to be OWNED BY ANOTHER OBJECT that shouldn't go out of
  // scope while this object exists.
  void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer);
  void setEsdfLayer(voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer);

  inline void setOptimistic(bool optimistic) { optimistic_ = optimistic; }
  bool getOptimistic() const { return optimistic_; }

  // Only call this once, only call this after setting all settings correctly.
  void setupProblem();

  // Fixed start and end locations, returns list of waypoints between.
  bool getPathBetweenWaypoints(
      const mav_msgs::EigenTrajectoryPoint& start,
      const mav_msgs::EigenTrajectoryPoint& goal,
      mav_msgs::EigenTrajectoryPoint::Vector* solution);

  void solutionPathToTrajectoryPoints(
      ompl::geometric::PathGeometric& path,
      mav_msgs::EigenTrajectoryPointVector* trajectory_points) const;

  // Even if planning fails, get the part of the tree that spans closest to
  // the original goal point. Returns true if it was actually successfully
  // able to plan to the original goal point, false otherwise.
  bool getBestPathTowardGoal(const mav_msgs::EigenTrajectoryPoint& start,
                             const mav_msgs::EigenTrajectoryPoint& goal,
                             mav_msgs::EigenTrajectoryPoint::Vector* solution);

 protected:
  void setupFromStartAndGoal(const mav_msgs::EigenTrajectoryPoint& start,
                             const mav_msgs::EigenTrajectoryPoint& goal);

  double getDistanceEigenToState(const Eigen::Vector3d& eigen,
                                 const ompl::base::State* state_ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Setup the problem in OMPL.
  ompl::mav::MavSetup problem_setup_;
  double num_seconds_to_plan_;
  bool simplify_solution_;
  double robot_radius_;
  bool verbose_;

  // Whether the planner is optimistic (true) or pessimistic (false) about
  // how unknown space is handled.
  // Optimistic uses the TSDF for collision checking, while pessimistic uses
  // the ESDF. Be sure to set the maps accordingly.
  bool optimistic_;

  // Whether to trust an approximate solution (i.e., not necessarily reaching
  // the exact goal state).
  bool trust_approx_solution_;

  // Planning bounds, if set.
  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;

  // NON-OWNED pointers to the relevant layers. TSDF only used if optimistic,
  // ESDF only used if pessimistic.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;

  double voxel_size_;
};

}  // namespace mav_planning

#endif  // RRT_PLANNER_VOXBLOX_OMPL_RRT_VOXBLOX_H_