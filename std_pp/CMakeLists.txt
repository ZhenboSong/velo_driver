cmake_minimum_required(VERSION 2.8.3)
project(std_pp)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
	std_msgs
    )
	
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mmm　
  CATKIN_DEPENDS  roscpp std_msgs 
  # geometry_msgs　nav_msgs
#  DEPENDS PCL OPENCV
#  DEPENDS system_lib
)

include_directories(include 
 
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
	# ${OPENCV_INCLUDE_DIR}
  #  ${OPENCV_INCLUDE_DIRS}
	# ${PCL_INCLUDE_DIRS})
)

add_executable(std_send
  src/std_send.cpp
)
add_dependencies(std_send ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(std_send
  ${catkin_LIBRARIES}
)

add_executable(std_rcv
  src/std_rcv.cpp
)
add_dependencies(std_rcv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(std_rcv
  ${catkin_LIBRARIES}
)
