cmake_minimum_required(VERSION 2.8.3)
project(loam_livox)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

#### Specify PCL version
if(NOT DEFINED ENV{PCL_ROOT})
    message(WARNING "not defined environment variable:PCL_ROOT")
else()
    if (POLICY CMP0074)
        cmake_policy(SET CMP0074 NEW) # CMake 3.12, Set policy CMP0074 explicitly to avoid warnings
    endif ()
    message(STATUS "***** PCL_ROOT = " $ENV{PCL_ROOT} "*****" )
#     set(PCL_DIR $ENV{PCL_ROOT})
#     set(PCL_DIR "/usr/local/share/pcl-1.8")
endif()
find_package(PCL REQUIRED)
message(STATUS "***** PCL version: ${PCL_VERSION} *****")
####

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)


add_executable(livox_scanRegistration src/laser_feature_extractor.cpp)
target_link_libraries(livox_scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(livox_laserMapping src/laser_mapping.cpp)
target_link_libraries(livox_laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

