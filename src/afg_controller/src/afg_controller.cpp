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

#include "afg_controller/afg_controller.hpp"

#include <algorithm>
#include <string>
#include <memory>
#include <utility>
#include <limits>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"

namespace afg_controller
{

void AFGController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  RCLCPP_INFO(logger_, "Configuring AFG Controller: %s", plugin_name_.c_str());

  // Declare and get parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".convergence_gain", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".flow_gain", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".boundary_layer", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist", rclcpp::ParameterValue(10.0));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".convergence_gain", convergence_gain_);
  node->get_parameter(plugin_name_ + ".flow_gain", flow_gain_);
  node->get_parameter(plugin_name_ + ".boundary_layer", boundary_layer_);
  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);

  speed_limit_ = desired_linear_vel_;
  speed_limit_percentage_ = false;

  RCLCPP_INFO(
    logger_, 
    "AFG Controller configured with parameters:\n"
    "  desired_linear_vel: %.2f m/s\n"
    "  convergence_gain: %.2f\n"
    "  flow_gain: %.2f\n"
    "  boundary_layer: %.2f m\n"
    "  lookahead_distance: %.2f m\n"
    "  max_angular_vel: %.2f rad/s",
    desired_linear_vel_, convergence_gain_, flow_gain_,
    boundary_layer_, lookahead_distance_, max_angular_vel_);
}

void AFGController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up AFG Controller");
  global_plan_.poses.clear();
  transformed_plan_.poses.clear();
}

void AFGController::activate()
{
  RCLCPP_INFO(logger_, "Activating AFG Controller");
}

void AFGController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating AFG Controller");
}

void AFGController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_DEBUG(
    logger_, "Received global plan with %zu poses",
    global_plan_.poses.size());
}

geometry_msgs::msg::TwistStamped AFGController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // Check if we have a valid plan
  if (global_plan_.poses.empty()) {
    throw std::runtime_error("No global plan available!");
  }

  // Transform global plan to robot frame
  try {
    transformed_plan_ = transformGlobalPlan(pose);
  } catch (const std::exception & e) {
    throw std::runtime_error(
      std::string("Failed to transform global plan: ") + e.what());
  }

  if (transformed_plan_.poses.size() < 2) {
    throw std::runtime_error("Transformed plan has insufficient points!");
  }

  // Find closest point on path
  geometry_msgs::msg::Point closest_point;
  size_t segment_idx;
  double cross_track_error = findClosestPointOnPath(pose, closest_point, segment_idx);

  RCLCPP_DEBUG(
    logger_, 
    "Cross-track error: %.3f m, segment: %zu/%zu",
    cross_track_error, segment_idx, transformed_plan_.poses.size());

  // Get path tangent
  geometry_msgs::msg::Vector3 tangent = getPathTangent(segment_idx, closest_point);

  // Compute flow field direction
  geometry_msgs::msg::Vector3 flow_direction = computeFlowField(
    pose, closest_point, tangent, cross_track_error);

  // Get current heading
  double current_heading = tf2::getYaw(pose.pose.orientation);

  // Compute control commands
  double linear_vel, angular_vel;
  computeControlCommands(flow_direction, current_heading, linear_vel, angular_vel);

  // Apply speed limit
  linear_vel = std::min(linear_vel, speed_limit_);

  // Create velocity command
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  RCLCPP_DEBUG(
    logger_,
    "Velocity commands: linear=%.2f m/s, angular=%.2f rad/s",
    linear_vel, angular_vel);

  return cmd_vel;
}

void AFGController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  speed_limit_percentage_ = percentage;

  if (percentage) {
    speed_limit_ = desired_linear_vel_ * (speed_limit / 100.0);
  } else {
    speed_limit_ = speed_limit;
  }

  RCLCPP_INFO(
    logger_, "Speed limit set to %.2f m/s (%s)",
    speed_limit_, percentage ? "percentage" : "absolute");
}

nav_msgs::msg::Path AFGController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw std::runtime_error("Global plan is empty!");
  }

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Get the robot's position in the global frame
  geometry_msgs::msg::PoseStamped robot_pose_global;
  try {
    tf_->transform(pose, robot_pose_global, global_plan_.header.frame_id,
                   tf2::durationFromSec(transform_tolerance_));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Could not transform robot pose: %s", ex.what());
    throw;
  }

  // Find the closest point on the path to start from
  double min_dist = std::numeric_limits<double>::max();
  size_t start_idx = 0;
  
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dist = distance(robot_pose_global.pose.position, 
                          global_plan_.poses[i].pose.position);
    if (dist < min_dist) {
      min_dist = dist;
      start_idx = i;
    }
  }

  // If robot is too far from path, start from beginning
  if (min_dist > max_robot_pose_search_dist_) {
    RCLCPP_WARN(
      logger_,
      "Robot is %.2f m from path (max: %.2f m), using path start",
      min_dist, max_robot_pose_search_dist_);
    start_idx = 0;
  }

  // Transform relevant portion of path to robot frame
  const std::string & target_frame = costmap_ros_->getBaseFrameID();
  
  for (size_t i = start_idx; i < global_plan_.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    try {
      tf_->transform(
        global_plan_.poses[i], transformed_pose, target_frame,
        tf2::durationFromSec(transform_tolerance_));
      transformed_plan.poses.push_back(transformed_pose);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger_,
        "Could not transform pose %zu: %s", i, ex.what());
      continue;
    }
  }

  if (transformed_plan.poses.empty()) {
    throw std::runtime_error("Failed to transform any poses!");
  }

  return transformed_plan;
}

double AFGController::findClosestPointOnPath(
  const geometry_msgs::msg::PoseStamped & pose,
  geometry_msgs::msg::Point & closest_point,
  size_t & segment_idx)
{
  double min_dist = std::numeric_limits<double>::max();
  closest_point = pose.pose.position;
  segment_idx = 0;

  // Robot position
  const auto & robot_pos = pose.pose.position;

  // Find closest segment and point on that segment
  for (size_t i = 0; i < transformed_plan_.poses.size() - 1; ++i) {
    const auto & p1 = transformed_plan_.poses[i].pose.position;
    const auto & p2 = transformed_plan_.poses[i + 1].pose.position;

    // Segment vector
    double segment_x = p2.x - p1.x;
    double segment_y = p2.y - p1.y;
    double segment_len_sq = segment_x * segment_x + segment_y * segment_y;

    if (segment_len_sq < 1e-12) {
      continue;  // Skip zero-length segments
    }

    // Project robot position onto line segment
    double pos_x = robot_pos.x - p1.x;
    double pos_y = robot_pos.y - p1.y;
    double dot_product = pos_x * segment_x + pos_y * segment_y;
    double projection = dot_product / segment_len_sq;

    // Clamp projection to segment
    projection = std::clamp(projection, 0.0, 1.0);

    // Point on segment
    geometry_msgs::msg::Point point_on_segment;
    point_on_segment.x = p1.x + projection * segment_x;
    point_on_segment.y = p1.y + projection * segment_y;
    point_on_segment.z = p1.z;

    // Distance to this point
    double dist = distance(robot_pos, point_on_segment);

    if (dist < min_dist) {
      min_dist = dist;
      closest_point = point_on_segment;
      segment_idx = i;
    }
  }

  return min_dist;
}

geometry_msgs::msg::Vector3 AFGController::getPathTangent(
  size_t segment_idx,
  const geometry_msgs::msg::Point & closest_point)
{
  geometry_msgs::msg::Vector3 tangent;
  tangent.x = 1.0;
  tangent.y = 0.0;
  tangent.z = 0.0;

  if (segment_idx >= transformed_plan_.poses.size() - 1) {
    segment_idx = transformed_plan_.poses.size() - 2;
  }

  // Use lookahead point for tangent calculation
  geometry_msgs::msg::Point lookahead_point;
  double cumulative_dist = 0.0;
  bool found_lookahead = false;

  for (size_t i = segment_idx; i < transformed_plan_.poses.size() - 1; ++i) {
    const auto & p1 = transformed_plan_.poses[i].pose.position;
    const auto & p2 = transformed_plan_.poses[i + 1].pose.position;
    double segment_length = distance(p1, p2);

    if (cumulative_dist + segment_length >= lookahead_distance_) {
      // Interpolate along this segment
      double remaining = lookahead_distance_ - cumulative_dist;
      double t = (segment_length > 0) ? (remaining / segment_length) : 0.0;
      
      lookahead_point.x = p1.x + t * (p2.x - p1.x);
      lookahead_point.y = p1.y + t * (p2.y - p1.y);
      lookahead_point.z = p1.z + t * (p2.z - p1.z);
      
      found_lookahead = true;
      break;
    }

    cumulative_dist += segment_length;
  }

  if (!found_lookahead) {
    // Use last point if lookahead distance exceeds path
    lookahead_point = transformed_plan_.poses.back().pose.position;
  }

  // Compute tangent from closest point to lookahead point
  double tangent_x = lookahead_point.x - closest_point.x;
  double tangent_y = lookahead_point.y - closest_point.y;
  double tangent_norm = std::sqrt(tangent_x * tangent_x + tangent_y * tangent_y);

  if (tangent_norm < 1e-6) {
    // Fallback to segment tangent
    const auto & p1 = transformed_plan_.poses[segment_idx].pose.position;
    const auto & p2 = transformed_plan_.poses[segment_idx + 1].pose.position;
    tangent_x = p2.x - p1.x;
    tangent_y = p2.y - p1.y;
    tangent_norm = std::sqrt(tangent_x * tangent_x + tangent_y * tangent_y);
  }

  if (tangent_norm > 1e-6) {
    tangent.x = tangent_x / tangent_norm;
    tangent.y = tangent_y / tangent_norm;
  }

  return tangent;
}

geometry_msgs::msg::Vector3 AFGController::computeFlowField(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Point & closest_point,
  const geometry_msgs::msg::Vector3 & tangent,
  double cross_track_error)
{
  geometry_msgs::msg::Vector3 flow_direction;

  // Error vector (from robot to closest point on path)
  double error_x = closest_point.x - pose.pose.position.x;
  double error_y = closest_point.y - pose.pose.position.y;

  // Apply boundary layer smoothing near path
  double convergence_strength = 1.0;
  if (cross_track_error < boundary_layer_) {
    // Linear interpolation within boundary layer
    convergence_strength = cross_track_error / boundary_layer_;
  }

  // Convergence component (towards path)
  double convergence_x = 0.0;
  double convergence_y = 0.0;
  
  if (cross_track_error > 1e-6) {
    convergence_x = convergence_gain_ * convergence_strength * (error_x / cross_track_error);
    convergence_y = convergence_gain_ * convergence_strength * (error_y / cross_track_error);
  }

  // Flow component (along path)
  double flow_x = flow_gain_ * tangent.x;
  double flow_y = flow_gain_ * tangent.y;

  // Combined flow field
  double flow_vec_x = convergence_x + flow_x;
  double flow_vec_y = convergence_y + flow_y;

  // Normalize to get direction
  double flow_norm = std::sqrt(flow_vec_x * flow_vec_x + flow_vec_y * flow_vec_y);
  
  if (flow_norm > 1e-6) {
    flow_direction.x = flow_vec_x / flow_norm;
    flow_direction.y = flow_vec_y / flow_norm;
  } else {
    // Fallback to tangent direction
    flow_direction.x = tangent.x;
    flow_direction.y = tangent.y;
  }
  
  flow_direction.z = 0.0;

  return flow_direction;
}

void AFGController::computeControlCommands(
  const geometry_msgs::msg::Vector3 & flow_direction,
  double current_heading,
  double & linear_vel,
  double & angular_vel)
{
  // Desired heading from flow field
  double desired_heading = std::atan2(flow_direction.y, flow_direction.x);

  // Heading error (wrapped to [-pi, pi])
  double heading_error = normalizeAngle(desired_heading - current_heading);

  // Angular velocity proportional to heading error
  angular_vel = 2.0 * heading_error;
  angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

  // Reduce linear velocity when turning sharply
  double heading_alignment = std::cos(heading_error);
  linear_vel = desired_linear_vel_ * std::max(0.3, heading_alignment);
}

}  // namespace afg_controller

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(afg_controller::AFGController, nav2_core::Controller)
