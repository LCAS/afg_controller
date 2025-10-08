# Artificial Flow Guidance (AFG) Controller for ROS2

A ROS2 implementation of an Artificial Flow Guidance controller for autonomous vehicle path following. This package provides **both a Python standalone node and a C++ Nav2 plugin** implementation.

## Overview

This package implements an AFG-based path following controller that uses vector field guidance to steer a vehicle along a desired trajectory. The controller generates smooth velocity commands by computing an artificial flow field around the reference path, combining convergence (towards the path) and circulation (along the path) components.

## Implementations

### 1. Python Standalone Node
- **File**: `afg_controller/afg_controller.py`
- **Use Case**: Testing, standalone operation, research/prototyping
- **Interface**: Standard ROS2 topics (`/odom`, `/desired_path`, `/cmd_vel`)
- **Features**: Full visualization support, easy to modify

### 2. C++ Nav2 Plugin
- **Files**: `src/afg_controller.cpp`, `include/afg_controller/afg_controller.hpp`
- **Use Case**: Production use with Nav2 navigation stack
- **Interface**: Nav2 controller plugin API
- **Features**: Drop-in replacement for DWB, TEB, or RPP controllers

## Quick Start

### For Nav2 Integration (Recommended)

See **[NAV2_INTEGRATION.md](NAV2_INTEGRATION.md)** for complete Nav2 setup instructions.

```bash
# Build the package
cd /workspaces/afg_controller
colcon build --packages-select afg_controller
source install/setup.bash

# Verify plugin registration
ros2 pkg plugins --package nav2_core --plugin nav2_core::Controller

# Use in Nav2 params file
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "afg_controller::AFGController"
      desired_linear_vel: 0.5
      convergence_gain: 1.5
      flow_gain: 2.0
```

### For Standalone Python Node

```bash
# Run the Python node
ros2 run afg_controller afg_controller

# Or with a launch file
ros2 launch afg_controller test_afg_controller.launch.py
```

## Algorithm Description

### Artificial Flow Guidance

Artificial Flow Guidance (AFG) is a motion planning method that uses velocity vectors to guide vehicle motion. The approach is based on vector field guidance principles, where an artificial flow field is constructed around a desired path.

The flow field consists of two main components:

1. **Convergence Component**: Generates vectors pointing towards the path, reducing cross-track error
2. **Flow Component**: Generates vectors tangent to the path, guiding the vehicle along the desired trajectory

### Mathematical Formulation

At any position **p**, the flow field vector **v** is computed as:

```
v = k_conv * (p_closest - p) / |p_closest - p| + k_flow * t
```

Where:
- `k_conv`: Convergence gain (controls how strongly the vehicle is pulled towards the path)
- `k_flow`: Flow gain (controls how strongly the vehicle follows the path tangent)
- `p_closest`: Closest point on the path
- `t`: Unit tangent vector along the path
- `|·|`: Euclidean norm

### Boundary Layer

To avoid singularities and provide smooth control near the path, a boundary layer is implemented:

```
if cross_track_error < boundary_layer:
    convergence_strength = cross_track_error / boundary_layer
else:
    convergence_strength = 1.0
```

This provides linear interpolation within the boundary layer, preventing abrupt control changes when the vehicle is very close to the path.

### Lookahead Strategy

The path tangent is computed using a lookahead distance rather than the immediate local tangent. This provides smoother control and helps prevent oscillations, particularly on paths with varying curvature.

## Features

- **Real-time path following**: Continuously computes control commands based on vehicle odometry and desired path
- **Configurable parameters**: All controller gains and parameters can be adjusted via ROS2 parameters
- **Flow field visualisation**: Publishes marker arrays for visualisation in RViz
- **Debug information**: Displays cross-track error, closest point, and current flow direction
- **Smooth control**: Implements boundary layer smoothing and speed reduction during sharp turns

## Installation

### Dependencies

```bash
# ROS2 (tested on Humble/Iron)
sudo apt install ros-humble-rclpy ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-visualization-msgs
```

### Build

```bash
cd ~/ros2_ws/src
# Clone or copy afg_controller package
cd ~/ros2_ws
colcon build --packages-select afg_controller
source install/setup.bash
```

## Usage

### Basic Usage

```bash
ros2 run afg_controller afg_controller
```

### With Custom Parameters

```bash
ros2 run afg_controller afg_controller --ros-args -p desired_speed:=0.8 -p convergence_gain:=1.5
```

### Parameter File

Create a `afg_params.yaml` file:

```yaml
afg_controller:
  ros__parameters:
    desired_speed: 0.8
    convergence_gain: 1.5
    flow_gain: 2.5
    boundary_layer: 0.15
    lookahead_distance: 0.7
    max_angular_vel: 1.2
    update_rate: 20.0
    viz_grid_resolution: 0.5
    viz_grid_size: 6.0
    viz_arrow_scale: 0.4
```

Launch with parameters:

```bash
ros2 run afg_controller afg_controller --ros-args --params-file afg_params.yaml
```

## ROS2 Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Vehicle odometry (position and orientation) |
| `/desired_path` | `nav_msgs/Path` | Desired path to follow |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for the vehicle |
| `/flow_field_viz` | `visualization_msgs/MarkerArray` | Flow field visualisation (grid of arrows) |
| `/afg_debug` | `visualization_msgs/MarkerArray` | Debug markers (closest point, cross-track error line, flow direction) |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `desired_speed` | double | 0.5 | Desired linear velocity (m/s) |
| `convergence_gain` | double | 1.5 | Gain for convergence component |
| `flow_gain` | double | 2.0 | Gain for flow component |
| `boundary_layer` | double | 0.1 | Boundary layer width (m) |
| `lookahead_distance` | double | 0.5 | Lookahead distance for tangent calculation (m) |
| `max_angular_vel` | double | 1.0 | Maximum angular velocity (rad/s) |
| `update_rate` | double | 20.0 | Control loop frequency (Hz) |
| `viz_grid_resolution` | double | 0.5 | Spacing between flow field arrows (m) |
| `viz_grid_size` | double | 5.0 | Half-size of visualisation grid (m) |
| `viz_arrow_scale` | double | 0.3 | Scale factor for visualisation arrows |

## Visualisation in RViz

### Setup

1. Launch RViz:
```bash
rviz2
```

2. Add the following displays:
   - **MarkerArray** → Topic: `/flow_field_viz`
     - Shows the flow field as a grid of coloured arrows
     - Green arrows: Near the path
     - Red arrows: Far from the path
   
   - **MarkerArray** → Topic: `/afg_debug`
     - Orange sphere: Closest point on path
     - Red line: Cross-track error
     - Cyan arrow: Current flow direction at vehicle
   
   - **Path** → Topic: `/desired_path`
     - Shows the reference trajectory
   
   - **Odometry** → Topic: `/odom`
     - Shows vehicle position and orientation

3. Set the fixed frame to `map` or your odometry frame

### Visualisation Interpretation

- **Flow Field Arrows**: Show the desired velocity direction at each point in space
  - Length indicates relative magnitude
  - Direction shows where the flow would guide the vehicle
  - Colour indicates distance from path (green = on path, red = far)

- **Debug Markers**:
  - The orange sphere moves along the path showing the closest point
  - The red line visualises the cross-track error
  - The cyan arrow shows the actual flow direction being used for control

## Tuning Guide

### Convergence Gain (`convergence_gain`)

- **Higher values**: Faster convergence to path, but may cause oscillations
- **Lower values**: Smoother approach, but slower convergence
- **Recommended range**: 1.0 - 3.0

### Flow Gain (`flow_gain`)

- **Higher values**: Stronger preference for following path direction
- **Lower values**: More influenced by convergence component
- **Recommended range**: 1.5 - 4.0

### Boundary Layer (`boundary_layer`)

- **Larger values**: Smoother control near path, but less precise tracking
- **Smaller values**: Tighter tracking, but may cause jitter
- **Recommended range**: 0.05 - 0.3 m

### Lookahead Distance (`lookahead_distance`)

- **Larger values**: Smoother on curved paths, but less responsive
- **Smaller values**: More responsive, but may oscillate on curves
- **Recommended**: 2-3 times the vehicle wheelbase or width

## Example: Publishing a Test Path

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(Path, '/desired_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)
        
    def publish_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Create a sinusoidal path
        for s in np.linspace(0, 10, 100):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = s
            pose.pose.position.y = 2.0 * np.sin(s)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self.publisher.publish(path)
        self.get_logger().info('Published test path')

def main():
    rclpy.init()
    node = PathPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Theoretical Background & References

This implementation is based on vector field guidance methods used in autonomous vehicle control. The core concepts draw from the following areas:

### Key Concepts

1. **Vector Field Path Following**: Using artificial potential fields or flow fields to guide vehicles along desired trajectories
2. **Lyapunov Vector Fields**: Ensuring convergence through proper vector field construction
3. **Gradient Vector Fields**: Combining convergence and circulation components for path following

### Research Resources

The implementation is inspired by research in the following areas:

- **Artificial Flow Guidance for Articulated Vehicles**:
  - [ResearchGate Paper on AFG for Articulated Vehicles](https://www.researchgate.net/publication/376164320_Artificial_Flow_Guidance_Method_for_Combined_Off-Tracking_and_Stability_Improvement_for_Automated_Articulated_Vehicles)
  - Describes AFG as a motion planning method using velocity vectors with boundary layer modifications

- **Vector Field Guidance Methods**:
  - [AIAA Paper: Vector Field UAV Guidance](https://arc.aiaa.org/doi/10.2514/1.G004053)
  - Covers gradient vector fields (GVF) with convergence and circulation components
  
  - [IEEE Paper: Guiding Vector-Field Algorithm](https://www.researchgate.net/publication/309191959_A_Guiding_Vector-Field_Algorithm_for_Path-Following_Control_of_Nonholonomic_Mobile_Robots)
  - Provides theoretical foundations for vector field path following

- **Switched Vector Field Approaches**:
  - [ArXiv: Switched Vector Field-based Guidance](https://arxiv.org/abs/2405.06355)
  - Discusses adaptive vector field structures based on cross-track error

- **Marine Vehicle Applications**:
  - [SAGE Journals: Intelligent Vector-Based Path Following for USVs](https://journals.sagepub.com/doi/10.1177/09544062241290703)
  - Demonstrates vector field methods for unmanned surface vehicles

### Related Techniques

- **Artificial Potential Fields (APF)**: Similar concept using attractive/repulsive potentials
- **Pure Pursuit Controller**: Simpler geometric path following (AFG generalises this)
- **Model Predictive Control (MPC)**: Optimisation-based alternative
- **Line-of-Sight (LOS) Guidance**: Marine guidance method with similar principles

## Applications

This controller is suitable for:

- **Ground robots**: Differential drive, car-like, and omnidirectional platforms
- **Autonomous vehicles**: Path tracking for autonomous cars
- **Marine vessels**: Surface vessel guidance (with appropriate modifications)
- **Warehouse AGVs**: Automated guided vehicle path following
- **Agricultural robots**: Row following and field navigation

## Limitations & Future Work

### Current Limitations

- Assumes 2D planar motion (no elevation changes)
- No explicit obstacle avoidance (path assumed to be collision-free)
- Constant desired speed (no speed optimisation)
- Single path tracking (no path switching or replanning)

### Potential Extensions

- **3D Extension**: Extend to 3D for aerial or underwater vehicles
- **Dynamic Obstacles**: Integrate repulsive flow fields for obstacle avoidance
- **Adaptive Speed**: Adjust speed based on path curvature and tracking error
- **Multi-Vehicle Coordination**: Extend to formation control with multiple agents
- **Path Replanning**: Dynamic path updates and smooth transitions
- **Predictive Control**: Incorporate model predictive elements for constraint handling

## Troubleshooting

### Vehicle oscillates around path

- Reduce `convergence_gain`
- Increase `boundary_layer`
- Increase `lookahead_distance`

### Vehicle doesn't converge to path

- Increase `convergence_gain`
- Check that `/desired_path` is being published
- Verify odometry frame matches path frame

### Visualisation not appearing in RViz

- Check that topics are being published: `ros2 topic list`
- Verify frame_id matches your setup
- Ensure MarkerArray displays are added in RViz

### Vehicle moves too slowly on curves

- Adjust the speed reduction factor in `compute_control_commands()`
- Increase `desired_speed`
- Tune angular velocity limits

## Citation

If you use this implementation in your research, please cite the relevant vector field guidance papers listed in the references section.

## Licence

Apache-2.0

## Contact

Marc Hanheide, LCAS - Lincoln Centre for Autonomous Systems
Email: marc@hanheide.net

---

**Note**: This is a research/educational implementation. For production use in safety-critical applications, additional validation, testing, and safety mechanisms should be implemented.
