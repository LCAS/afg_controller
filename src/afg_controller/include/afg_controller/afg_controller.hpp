// Copyright 2025 LCAS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AFG_CONTROLLER__AFG_CONTROLLER_HPP_
#define AFG_CONTROLLER__AFG_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace afg_controller
{

/**
 * @brief Artificial Flow Guidance (AFG) Controller for Nav2
 * 
 * This controller implements an AFG-based path following algorithm that uses
 * vector field guidance to generate smooth velocity commands for path tracking.
 * 
 * The AFG algorithm combines:
 * 1. Convergence component: Guides the robot towards the path
 * 2. Flow component: Guides the robot along the path
 * 
 * This creates a smooth flow field that ensures stable and predictable
 * path following behaviour.
 */
class AFGController : public nav2_core::Controller
{
public:
  AFGController() = default;
  ~AFGController() override = default;

  /**
   * @brief Configure controller on bringup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller on shutdown
   */
  void cleanup() override;

  /**
   * @brief Activate controller
   */
  void activate() override;

  /**
   * @brief Deactivate controller
   */
  void deactivate() override;

  /**
   * @brief Set the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Compute velocity commands
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to goal checker
   * @return TwistStamped velocity command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief Set speed limit
   * @param speed_limit Speed limit to set
   * @param percentage Whether speed_limit is a percentage
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transform global plan to local frame
   * @param pose Current robot pose
   * @return Transformed path in robot's frame
   */
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Find closest point on path and compute cross-track error
   * @param pose Current robot pose
   * @param closest_point Output: closest point on path
   * @param segment_idx Output: index of closest path segment
   * @return Cross-track error (distance to path)
   */
  double findClosestPointOnPath(
    const geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::Point & closest_point,
    size_t & segment_idx);

  /**
   * @brief Get path tangent at closest point using lookahead
   * @param segment_idx Index of path segment
   * @param closest_point Closest point on path
   * @return Unit tangent vector along path
   */
  geometry_msgs::msg::Vector3 getPathTangent(
    size_t segment_idx,
    const geometry_msgs::msg::Point & closest_point);

  /**
   * @brief Compute artificial flow field vector at current position
   * @param pose Current robot pose
   * @param closest_point Closest point on path
   * @param tangent Path tangent vector
   * @param cross_track_error Distance to path
   * @return Flow direction vector (normalized)
   */
  geometry_msgs::msg::Vector3 computeFlowField(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Point & closest_point,
    const geometry_msgs::msg::Vector3 & tangent,
    double cross_track_error);

  /**
   * @brief Compute linear and angular velocity commands
   * @param flow_direction Desired heading direction from flow field
   * @param current_heading Current robot heading
   * @param linear_vel Output: linear velocity command
   * @param angular_vel Output: angular velocity command
   */
  void computeControlCommands(
    const geometry_msgs::msg::Vector3 & flow_direction,
    double current_heading,
    double & linear_vel,
    double & angular_vel);

  /**
   * @brief Calculate distance between two points
   */
  inline double distance(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2)
  {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  /**
   * @brief Normalize an angle to [-pi, pi]
   */
  inline double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  // Member variables
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("AFGController")};
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path transformed_plan_;

  // AFG Parameters
  double desired_linear_vel_;      // Desired forward velocity (m/s)
  double convergence_gain_;        // Gain for convergence to path
  double flow_gain_;               // Gain for flow along path
  double boundary_layer_;          // Boundary layer width (m)
  double lookahead_distance_;      // Lookahead distance for tangent (m)
  double max_angular_vel_;         // Maximum angular velocity (rad/s)
  double transform_tolerance_;     // Transform lookup tolerance (s)
  double max_robot_pose_search_dist_;  // Max distance to search for poses
  
  // Speed limit
  double speed_limit_;
  bool speed_limit_percentage_;
};

}  // namespace afg_controller

#endif  // AFG_CONTROLLER__AFG_CONTROLLER_HPP_
