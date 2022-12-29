cmake_minimum_required(VERSION 3.5)
project(toroboeye_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY
  launch
  urdf
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
