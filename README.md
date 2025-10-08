# AFG Controller

A ROS2 workspace containing an Artificial Flow Guidance (AFG) controller for autonomous vehicle path following.

## Overview

This repository provides a ROS2 implementation of an AFG-based path following controller with both Python and C++ implementations. The controller uses vector field guidance to steer vehicles along desired trajectories.

## Packages

This workspace contains the following ROS2 packages in the `src/` directory:

- **[afg_controller](src/afg_controller/README.md)**: The main AFG controller package with Python standalone node and C++ Nav2 plugin implementations

## Quick Start

```bash
# Clone the repository
git clone https://github.com/LCAS/afg_controller.git
cd afg_controller

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Run the controller (Python standalone)
ros2 run afg_controller afg_controller
```

For detailed installation instructions, usage examples, parameter tuning, and Nav2 integration, please refer to the **[afg_controller package README](src/afg_controller/README.md)**.

## Development

This repository is configured with a devcontainer for easy development. Open the repository in Visual Studio Code and select "Reopen in Container" to get started with a fully configured ROS2 environment.
