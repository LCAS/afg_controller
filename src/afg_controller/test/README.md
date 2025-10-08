# AFG Controller Test Suite

This directory contains comprehensive tests for the Artificial Flow Guidance (AFG) Controller.

## Test Structure

### Unit Tests (`TestAFGController`)
- **test_initialization**: Verifies controller initialization and parameter setting
- **test_path_callback**: Tests path message handling and waypoint extraction
- **test_closest_point_on_straight_path**: Tests closest point computation on straight paths
- **test_closest_point_on_curved_path**: Tests closest point computation on curved paths
- **test_path_tangent_computation**: Tests path tangent vector calculation
- **test_flow_field_computation**: Tests artificial flow field generation
- **test_control_commands_computation**: Tests velocity command generation
- **test_boundary_layer_behavior**: Tests smooth convergence behavior near paths
- **test_angular_velocity_limits**: Tests angular velocity limiting
- **test_empty_path_handling**: Tests behavior with invalid/empty paths
- **test_odometry_callback**: Tests odometry message handling

### Integration Tests (`TestAFGControllerIntegration`)
- **test_full_control_loop**: Tests complete control loop execution
- **test_visualization_publishing**: Tests visualization message publishing

## Running Tests

### Method 1: Using colcon (Recommended)
```bash
cd /path/to/afg_controller
colcon test --packages-select afg_controller
colcon test-result --verbose
### Running specific tests
```bash
# Run only unit tests
python -m pytest test/test_afg_controller.py::TestAFGController -v

# Run only integration tests
python -m pytest test/test_afg_controller.py::TestAFGControllerIntegration -v

# Run a specific test
python -m pytest test/test_afg_controller.py::TestAFGController::test_flow_field_computation -v
```

## Test Data Generator

The `test_data_generator.py` script can create various path types for testing:

```bash
# Generate a straight path
ros2 run afg_controller test_data_generator --path-type straight

# Generate a circular path
ros2 run afg_controller test_data_generator --path-type circle

# Generate a sinusoidal path
ros2 run afg_controller test_data_generator --path-type sine

# Generate a figure-eight path
ros2 run afg_controller test_data_generator --path-type figure8
```

## Manual Testing with Launch File

Use the provided launch file for manual testing:

```bash
ros2 launch afg_controller test_afg_controller.launch.py
```

Launch parameters:
- `desired_speed`: Target linear velocity (default: 0.5 m/s)
- `convergence_gain`: Convergence strength (default: 1.5)
- `flow_gain`: Flow strength along path (default: 2.0)
- `use_sim_time`: Use simulation time (default: false)

## Test Coverage

The test suite covers:
- ✅ Path processing and waypoint extraction
- ✅ Closest point computation on various path geometries
- ✅ Flow field generation and convergence behavior
- ✅ Control command computation and velocity limiting
- ✅ Boundary layer smoothing
- ✅ Edge cases (empty paths, invalid data)
- ✅ ROS2 message handling
- ✅ Complete control loop integration

## Expected Test Results

All 13 tests should pass:
- 11 unit tests covering core algorithms
- 2 integration tests covering ROS2 integration

## Test Dependencies

- `rclpy`: ROS2 Python client library
- `numpy`: Numerical computations
- `pytest`: Test framework
- Standard ROS2 message packages (`geometry_msgs`, `nav_msgs`, etc.)

## Troubleshooting

If tests fail:
1. Ensure ROS2 is properly sourced
2. Check that all dependencies are installed
3. Verify the package builds successfully with `colcon build`
4. Use `python -m pytest` instead of `colcon test` if import issues occur