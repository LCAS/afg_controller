# AFG Controller - Nav2 Integration

This directory contains the Nav2 C++ plugin implementation of the Artificial Flow Guidance (AFG) controller.

## Overview

The AFG Controller is implemented as a Nav2 controller plugin that can be used as a drop-in replacement for other Nav2 controllers like DWB, TEB, or RPP. It uses vector field guidance to generate smooth velocity commands for path following.

## Architecture

The package provides two implementations:
1. **Python Standalone Node** (`afg_controller.py`): For testing and standalone operation
2. **C++ Nav2 Plugin** (`src/afg_controller.cpp`): For production use with Nav2

### C++ Plugin Files

```
include/afg_controller/
└── afg_controller.hpp          # Controller header
src/
└── afg_controller.cpp          # Controller implementation
plugins/
└── afg_controller_plugin.xml   # Plugin description
```

## Building

This is a hybrid package that builds both Python and C++ components:

```bash
cd /workspaces/afg_controller
colcon build --packages-select afg_controller
source install/setup.bash
```

## Verify Plugin Registration

After building, verify the plugin is registered:

```bash
ros2 pkg plugins --package nav2_core --plugin nav2_core::Controller
```

You should see `afg_controller::AFGController` in the output.

## Configuration

### Nav2 Parameters

Add the AFG controller to your Nav2 parameters file:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]
    
    # Progress checker
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    
    # Goal checker
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true
    
    # AFG Controller configuration
    FollowPath:
      plugin: "afg_controller::AFGController"
      desired_linear_vel: 0.5          # m/s - desired forward velocity
      convergence_gain: 1.5             # gain for convergence to path
      flow_gain: 2.0                    # gain for flow along path
      boundary_layer: 0.1               # m - boundary layer width
      lookahead_distance: 0.5           # m - lookahead for tangent calculation
      max_angular_vel: 1.0              # rad/s - maximum angular velocity
      transform_tolerance: 0.1          # s - transform lookup tolerance
      max_robot_pose_search_dist: 10.0  # m - max distance to search for robot on path
```

### Multiple Controllers

You can configure multiple controllers for different scenarios:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["AFGFollowPath", "DWBFollowPath"]
    
    # AFG for smooth outdoor paths
    AFGFollowPath:
      plugin: "afg_controller::AFGController"
      desired_linear_vel: 1.0
      convergence_gain: 2.0
      flow_gain: 3.0
      lookahead_distance: 1.0
    
    # DWB for crowded indoor environments
    DWBFollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # DWB parameters...
```

## Parameter Descriptions

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `desired_linear_vel` | double | 0.5 | Desired forward velocity (m/s) |
| `convergence_gain` | double | 1.5 | Gain for convergence component towards path |
| `flow_gain` | double | 2.0 | Gain for flow component along path |
| `boundary_layer` | double | 0.1 | Boundary layer width for smooth transitions (m) |
| `lookahead_distance` | double | 0.5 | Lookahead distance for tangent calculation (m) |
| `max_angular_vel` | double | 1.0 | Maximum angular velocity (rad/s) |
| `transform_tolerance` | double | 0.1 | TF transform lookup tolerance (s) |
| `max_robot_pose_search_dist` | double | 10.0 | Maximum distance to search for robot pose on path (m) |

## Usage with Nav2

### Launch Nav2 with AFG Controller

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/path/to/your/nav2_params.yaml
```

### Runtime Parameter Updates

```bash
# Get current parameters
ros2 param list /controller_server | grep FollowPath

# Update a parameter
ros2 param set /controller_server FollowPath.desired_linear_vel 0.8

# Get a specific parameter
ros2 param get /controller_server FollowPath.convergence_gain
```

### Dynamic Speed Limiting

The AFG controller supports Nav2's speed limiting interface:

```python
# Via service call
ros2 service call /controller_server/set_speed_limit \
  nav2_msgs/srv/SetSpeed \
  "{speed: 0.3, percentage: false}"

# Percentage-based
ros2 service call /controller_server/set_speed_limit \
  nav2_msgs/srv/SetSpeed \
  "{speed: 50.0, percentage: true}"
```

## Behaviour Tree Integration

Use the AFG controller in Nav2 behaviour trees:

```xml
<BehaviorTree ID="FollowPath">
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
      <RecoveryNode number_of_retries="1" name="ComputePathToPose">
        <ComputePathToPose goal="{goal}" path="{path}"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
      </RecoveryNode>
    </RateController>
    <RecoveryNode number_of_retries="1" name="FollowPath">
      <!-- Use AFG Controller -->
      <FollowPath path="{path}" controller_id="AFGFollowPath"/>
      <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
    </RecoveryNode>
  </PipelineSequence>
</BehaviorTree>
```

## Algorithm Overview

The AFG controller implements a vector field guidance approach:

1. **Path Transformation**: Global path is transformed to the robot's local frame
2. **Closest Point**: Find the closest point on the path to the robot
3. **Tangent Calculation**: Compute path tangent using lookahead distance
4. **Flow Field**: Combine convergence (towards path) and flow (along path) components
5. **Velocity Commands**: Generate linear and angular velocities based on flow direction

### Flow Field Equation

```
v_flow = k_conv * (convergence_component) + k_flow * (tangent_component)
```

Where:
- Convergence component guides the robot towards the path
- Flow component guides the robot along the path
- Gains balance between convergence and progression

## Comparison with Other Controllers

| Controller | Strengths | Best Use Cases |
|------------|-----------|----------------|
| **AFG** | Smooth paths, predictable, efficient | Outdoor, corridors, known environments |
| **DWB** | Obstacle avoidance, dynamic environments | Crowded spaces, unknown environments |
| **RPP** | Simple, efficient for car-like robots | Warehouses, structured environments |
| **MPPI** | Optimal control, handles constraints | Complex dynamics, optimization needed |
| **TEB** | Time-optimal, considers dynamics | Time-critical tasks, car-like robots |

## Debugging

### Enable Debug Logging

```bash
ros2 run nav2_controller controller_server --ros-args --log-level DEBUG
```

### Check Plugin Loading

```bash
# View controller server logs
ros2 topic echo /rosout | grep AFG

# Check if plugin is loaded
ros2 lifecycle get /controller_server
```

### Common Issues

1. **Plugin not found**: Ensure package is built and sourced
   ```bash
   colcon build --packages-select afg_controller
   source install/setup.bash
   ```

2. **Transform errors**: Check TF tree and increase `transform_tolerance`
   ```bash
   ros2 run tf2_tools view_frames
   ```

3. **Path following issues**: Adjust gains (`convergence_gain`, `flow_gain`)

## Performance Tuning

### High-Speed Operation
```yaml
desired_linear_vel: 2.0
lookahead_distance: 1.5
max_angular_vel: 2.0
```

### Tight Corridors
```yaml
convergence_gain: 3.0
boundary_layer: 0.05
lookahead_distance: 0.3
```

### Smooth Outdoor Paths
```yaml
flow_gain: 3.0
convergence_gain: 1.0
lookahead_distance: 2.0
```

## Testing

### Unit Tests (C++)
```bash
colcon test --packages-select afg_controller
colcon test-result --verbose
```

### Python Tests
```bash
pytest src/afg_controller/test/
```

## References

- [Nav2 Controller Tutorial](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html)
- [Nav2 Configuration Guide](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)
- [Nav2 Concepts](https://docs.nav2.org/concepts/index.html)

## License

Apache 2.0 - See LICENSE file
