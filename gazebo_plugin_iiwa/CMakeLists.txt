cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugin_iiwa)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  
## servono per kdl 
  geometry_msgs
  kdl_parser
  sensor_msgs
#  orocos_kdl
#  rtt_ros
)

find_package(orocos_kdl REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${USE_OROCOS_INCLUDE_DIRS}
#  ${OROCOS_KDL_INCLUDE_DIRS}
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
#include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${OROCOS_KDL_INCLUDE_DIRS})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    
#    rtt_ros

)



#add_library(control_iiwa src/control.cpp)
#target_link_libraries(control_iiwa ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
	pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(plugin_control_iiwa SHARED src/control.cpp)
target_link_libraries(plugin_control_iiwa ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


#target_link_libraries(plugin_control_iiwa ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OROCOS_KDL_LIBRARIES} ${USE_OROCOS_LIBRARIES})


#target_link_libraries(${PROJECT_NAME} ${OROCOS_KDL_LIBRARIES} ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})



