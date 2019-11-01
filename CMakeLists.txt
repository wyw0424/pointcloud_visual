cmake_minimum_required(VERSION 2.8.3)
project(m6_runtime_visual)

find_package(catkin REQUIRED COMPONENTS
			roscpp
			pcl_ros
			tf
			std_srvs
			cv_bridge
			laser_geometry
			sensor_msgs
			geometry_msgs
			nav_msgs
			visualization_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenMP REQUIRED)
if(PCL_FOUND)
    #set(PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.8")
    #set(PCL_LIBRARY_DIRS "/usr/local/lib")
    message(STATUS "Using PCL PATH:" "\n" 
    		${PCL_INCLUDE_DIRS} "\n" 
    		${PCL_LIBRARY_DIRS}"\n")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif(PCL_FOUND)
###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
find_package(Eigen3 REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(visual_node src/visual_node.cpp)
target_link_libraries(visual_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


#############
## Install ##
#############

 install(TARGETS visual_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# install(DIRECTORY common/include
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

set(CMAKE_CXX_FLAGS "-fpermissive -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")  
