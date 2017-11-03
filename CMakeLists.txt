cmake_minimum_required(VERSION 2.8.3)
project(velo_driver)

set(${PROJECT_NAME}_CATKIN_DEPS 
    nodelet
    roscpp
    tf
    angles
    nodelet
    pcl_ros
    roslib
    sensor_msgs
    )

find_package(catkin REQUIRED COMPONENTS
  roscpp ${${PROJECT_NAME}_CATKIN_DEPS} pcl_conversions)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(
  include ${catkin_INCLUDE_DIRS}include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# objects needed by other ROS packages that depend on this one
catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
               INCLUDE_DIRS include)
               #LIBRARIES velodyne_input)


# add_library(velodyne_input  src/input.cpp src/calibration.cpp)
# target_link_libraries(velodyne_input
#   ${catkin_LIBRARIES}
#   ${libpcap_LIBRARIES}
# )
# if(catkin_EXPORTED_TARGETS)
#   add_dependencies(velodyne_input ${catkin_EXPORTED_TARGETS})
# endif()

# install(TARGETS velodyne_input
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )


# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# install(FILES nodelet_velodyne.xml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(DIRECTORY launch/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
# install(PROGRAMS src/vdump
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  


add_executable(output
  src/output.cpp
  src/calibration.cpp
  src/input.cpp
)
add_dependencies(output ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(output
  ${catkin_LIBRARIES}  ${YAML_CPP_LIBRARIES}
)