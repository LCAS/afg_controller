# Testing the AFG Controller Nav2 Plugin

This guide explains how to test the AFG Controller as a Nav2 plugin.

## Prerequisites

1. **ROS2 Installed**: Humble or newer
2. **Nav2 Installed**: `sudo apt install ros-${ROS_DISTRO}-navigation2`
3. **Package Built**:
   ```bash
   cd /workspaces/afg_controller
   colcon build --packages-select afg_controller
   source install/setup.bash
   ```

## Quick Verification

### 1. Check Plugin Registration

Verify the plugin XML is properly exported:

```bash
source install/setup.bash
cat install/afg_controller/share/afg_controller/package.xml | grep nav2_core
```

You should see:
```xml
<nav2_core plugin="${prefix}/plugins/afg_controller_plugin.xml" />
```

### 2. Check Library Installation

```bash
ls -la install/afg_controller/lib/libafg_controller.so
```

Should show the compiled shared library.

### 3. Check Plugin Description

```bash
cat install/afg_controller/share/afg_controller/plugins/afg_controller_plugin.xml
```

## Testing with Nav2 Controller Server

### Option 1: Minimal Test (Controller Server Only)

This tests just the controller server with AFG plugin:

```bash
# Terminal 1: Launch controller server with AFG
ros2 launch afg_controller nav2_controller_with_afg.launch.py

# Terminal 2: Check that it's running
ros2 lifecycle get /controller_server

# Terminal 3: Publish a test path (see below)
ros2 topic pub /plan nav_msgs/msg/Path "{...}"
```

### Option 2: Full Nav2 Stack

To test with the complete Nav2 stack:

1. Create a Nav2 params file that includes AFG controller configuration
2. Launch Nav2:

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/path/to/your/nav2_params.yaml
```

## Example: Standalone Controller Server Test

### 1. Start the Controller Server

```bash
source install/setup.bash
ros2 run nav2_controller controller_server \
  --ros-args \
  --params-file src/afg_controller/config/afg_controller_params.yaml
```

### 2. Check Lifecycle State

```bash
# Get current state
ros2 lifecycle get /controller_server

# Configure
ros2 lifecycle set /controller_server configure

# Activate
ros2 lifecycle set /controller_server activate
```

### 3. Monitor Topics

```bash
# Check available topics
ros2 topic list | grep controller

# Monitor velocity commands
ros2 topic echo /cmd_vel
```

## Integration Test with TurtleBot3 Simulation

If you have TurtleBot3 simulation available:

### 1. Launch TurtleBot3 World

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Launch Nav2 with AFG Controller

Create `nav2_afg_params.yaml` based on TurtleBot3 params but change controller:

```yaml
controller_server:
  ros__parameters:
    # ... other params ...
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "afg_controller::AFGController"
      desired_linear_vel: 0.22  # TurtleBot3 max linear vel
      max_angular_vel: 2.84      # TurtleBot3 max angular vel
      convergence_gain: 2.0
      flow_gain: 2.5
      boundary_layer: 0.1
      lookahead_distance: 0.5
```

Then launch:

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/path/to/nav2_afg_params.yaml
```

### 3. Set Navigation Goal in RViz

1. Click "2D Pose Estimate" to set initial pose
2. Click "Nav2 Goal" to set a navigation goal
3. Watch the robot follow the path using AFG controller!

## Debugging

### Enable Debug Logging

```bash
ros2 run nav2_controller controller_server \
  --ros-args \
  --params-file config/afg_controller_params.yaml \
  --log-level DEBUG
```

### Check for Errors

```bash
# Monitor rosout for errors
ros2 topic echo /rosout --field msg | grep -i "afg\|error"

# Check TF tree
ros2 run tf2_tools view_frames
```

### Verify Plugin Loading

Check controller server logs at startup. You should see:

```
[controller_server]: Configuring controller interface
[controller_server]: Creating controller plugin FollowPath of type afg_controller::AFGController
[controller_server]: AFG Controller configured with parameters:
[controller_server]:   desired_linear_vel: 0.50 m/s
[controller_server]:   convergence_gain: 1.50
...
```

## Common Issues

### Issue: "Failed to load plugin"

**Solution**: Ensure package is built and sourced:
```bash
colcon build --packages-select afg_controller
source install/setup.bash
```

### Issue: "Transform timeout"

**Solution**: 
- Check TF tree is publishing all required frames
- Increase `transform_tolerance` parameter
- Verify `odom` and `base_link` frames exist

### Issue: "No valid control"

**Solution**:
- Ensure global plan is being published to controller server
- Check that planner is generating valid paths
- Verify path frame_id matches expected frame

### Issue: Robot doesn't move

**Solution**:
- Check `/cmd_vel` topic is being published: `ros2 topic hz /cmd_vel`
- Verify robot is not in a recovery behavior
- Check velocity limits in parameters

## Performance Metrics

To evaluate AFG controller performance:

### 1. Cross-Track Error

Monitor how well the robot follows the path:

```bash
# Log controller output
ros2 topic echo /rosout | grep "Cross-track error"
```

### 2. Velocity Commands

Analyze smoothness of velocity commands:

```bash
# Record velocity commands
ros2 bag record /cmd_vel -o afg_test

# Play back and analyze
ros2 bag play afg_test.db3
```

### 3. Path Completion Time

Compare with other controllers (DWB, TEB, RPP) on the same path.

## Parameter Tuning Test Cases

Test different parameter sets for various scenarios:

### Scenario 1: High-Speed Straight Paths

```yaml
desired_linear_vel: 1.5
convergence_gain: 1.0
flow_gain: 3.0
lookahead_distance: 2.0
```

### Scenario 2: Tight Indoor Corridors

```yaml
desired_linear_vel: 0.3
convergence_gain: 3.0
flow_gain: 1.5
boundary_layer: 0.05
lookahead_distance: 0.3
```

### Scenario 3: Curved Outdoor Paths

```yaml
desired_linear_vel: 0.8
convergence_gain: 2.0
flow_gain: 2.5
lookahead_distance: 1.0
```

## Comparison Testing

To compare AFG with other controllers:

1. Record the same navigation task with different controllers
2. Measure:
   - Path completion time
   - Average cross-track error
   - Velocity smoothness (jerk)
   - Success rate

```bash
# Test with AFG
ros2 bag record -a -o test_afg

# Change params to DWB and repeat
ros2 bag record -a -o test_dwb

# Analyze and compare results
```

## Next Steps

After successful testing:

1. ✅ Verify plugin loads correctly
2. ✅ Test with simple paths
3. ✅ Test with complex curved paths
4. ✅ Compare performance with other controllers
5. ✅ Integrate with full Nav2 stack
6. ✅ Test in simulation
7. ✅ Test on real robot
8. ✅ Document optimal parameter sets for your robot

## References

- [Nav2 Controller Server](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
- [Nav2 Plugin Tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html)
- AFG Controller Documentation: `NAV2_INTEGRATION.md`
