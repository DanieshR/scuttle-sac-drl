cmake_minimum_required(VERSION 3.8)
project(scuttle_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. Find ROS 2 Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz REQUIRED) 
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)

# --- 2. Find Gazebo Harmonic Dependencies (By Version) ---
# In Jazzy/Noble, we must look for the specific versions
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# --- 3. Build the Gas Source Plugin ---
add_library(GasSourcePlugin SHARED src/GasSourcePlugin.cpp)

# Link against the versioned targets
target_link_libraries(GasSourcePlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# --- 4. Install Rules ---

# Install the Plugin Library
install(TARGETS GasSourcePlugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and worlds
install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/fix_scan_frame.py
  scripts/fix_odom_frame.py
  scripts/odom_to_tf.py
  scripts/teleop_autostop.py
  scripts/odom_reset_monitor.py
  scripts/gas_sensor_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()