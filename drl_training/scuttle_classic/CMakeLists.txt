cmake_minimum_required(VERSION 3.8)
project(scuttle_description)

find_package(ament_cmake REQUIRED)

# scuttle.urdf is pre-generated from scuttle.xacro and committed to the repo
install(DIRECTORY urdf meshes worlds launch models
  DESTINATION share/${PROJECT_NAME})

ament_package()
