cmake_minimum_required(VERSION 3.5)
project(summit_xl_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf robots vrep
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
