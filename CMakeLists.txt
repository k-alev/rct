cmake_minimum_required(VERSION 3.1)
# set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)
project(ros_control_toolbox)

#include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(rct_lib STATIC
  #include/kdl_wrap.h
  src/kdl_wrap.cpp
  src/kchain.cpp
  src/chainJntToJacDotSolver.cpp
	)
#target_include_directories(low_level_lib
#	PUBLIC include
#	PRIVATE src)

add_executable(test_main
	src/main.cpp
)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
#find_library(OROCOS_KDL orocos-kdl)
# link_directories(rct_lib /opt/ros/kinetic/lib) #giati me to link_diractories dn evriske thn lib to link libraries?

target_link_libraries(test_main 
  rct_lib
  ${orocos-kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)

target_include_directories(
  rct_lib
  PUBLIC
  /usr/include/eigen3
  ./include
  /opt/ros/kinetic/include
  )
