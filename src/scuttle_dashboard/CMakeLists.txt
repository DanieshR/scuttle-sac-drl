cmake_minimum_required(VERSION 3.8)
project(scuttle_dashboard)

find_package(ament_cmake REQUIRED)

# Install the launch files and the static web app (index.html, ros-bindings.js, themes/, vendor/).
install(
  DIRECTORY launch web
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
