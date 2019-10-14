cmake_minimum_required(VERSION 3.0.0)
project(fastseg)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        pcl_conversions
        )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        sensor_msgs
        std_msgs
        pcl_conversions
        pcl_ros
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


add_library(ground_plane_fitting
        src/ground_plane_fitting.cpp
        )


add_library(fastseg
        src/fastseg.cpp
        )

add_executable(fastseg_ros src/fastseg_ros.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fastseg_ros fastseg ground_plane_fitting ${catkin_LIBRARIES} ${PCL_LIBRARIES})
