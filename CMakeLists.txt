cmake_minimum_required(VERSION 2.8.3)
project(project_ri)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  xpp_vis
)

catkin_package()

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(urdf_project_ri src/exe/urdf_project_ri.cc)
target_link_libraries(urdf_project_ri
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS urdf_project_ri
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)