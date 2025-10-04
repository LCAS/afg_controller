cmake_minimum_required(VERSION 3.5)
project(afg_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# For Python node
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Set dependencies for C++ library
set(dependencies
  nav2_core
  nav2_costmap_2d
  nav2_util
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Create C++ library for Nav2 plugin
add_library(${PROJECT_NAME} SHARED
  src/afg_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Export plugin
pluginlib_export_plugin_description_file(nav2_core plugins/afg_controller_plugin.xml)

# Install C++ library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install C++ headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugin description
install(FILES plugins/afg_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}/plugins
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  afg_controller/afg_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install resource files
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# Export information
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  # C++ linting
  ament_lint_auto_find_test_dependencies()
  
  # Python testing
  ament_add_pytest_test(${PROJECT_NAME}_pytest test/
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
  )
endif()

ament_package()
