cmake_minimum_required(VERSION 2.8.3)
project(mapping)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  tf
  rosbag
  cv_bridge
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package( 
  CATKIN_DEPENDS roscpp tf 
  )

set(CMAKE_CXX_FLAGS "-ggdb -g -O3 -w -std=c++11 -fpermissive") # -fPIC -fpermissive
include_directories(/usr/include ${catkin_INCLUDE_DIRS})  
include_directories(/user/include/eigen3)
# include_directories("../cam_model/src")
# include_directories("../plane/src")

################ PCL #################
# set(PCL_DIR "/home/david/work/3rdlibs/pcl_1.7.1_src/pcl-pcl-1.7.1/build")
# set(PCL_DIR "/home/david/work/pcl/build")
# find_package(PCL 1.7 REQUIRED common io sample_consensus segmentation visualization)
# find_package(PCL 1.8 REQUIRED common io sample_consensus segmentation visualization geometry
#   PATHS "/usr/local/share/pcl-1.8")

find_package(PCL REQUIRED 
  PATHS "/usr/local/share/pcl-1.8")


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# message("STATUS catkin_LIBRARIES : ${catkin_LIBRARIES}")
# message("STATUS catkin_INCLUDE_DIRS : ${catkin_INCLUDE_DIRS}")

#############################
# Eigen ####################
#############################
set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3")
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
# add_definitions(-DEIGEN_DONT_ALIGN)

#############################
# OpenCV ####################
#############################
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#############################
# Qt ########################
# http://qtnode.net/wiki/Qt4_with_cmake
# find_package(Qt4 REQUIRED)
# SET(QT_USE_QTXML TRUE)
# SET(QT_USE_QTOPENGL TRUE)
# ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
# include(${QT_USE_FILE})

add_subdirectory(src)

