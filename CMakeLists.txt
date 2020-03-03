cmake_minimum_required(VERSION 3.1)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-fPIC")
project(ros_control_toolbox)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(rct_lib STATIC
  src/kdl_wrap.cpp
  src/kchain.cpp
  src/utils.cpp
  src/chainJntToJacDotSolver.cpp
	)

add_executable(test_main
	src/main.cpp
)

target_link_libraries(test_main PUBLIC
  rct_lib
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)

set(include_dest "include/${PROJECT_NAME}")
set(lib_dest_generic "lib/${PROJECT_NAME}")
set(lib_dest ${lib_dest_generic}/${CMAKE_BUILD_TYPE})

target_include_directories(rct_lib PUBLIC
                       $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include/rct> 
                       $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> 
                       $<BUILD_INTERFACE:${orocos_kdl_INCLUDE_DIRS}> 
                       $<BUILD_INTERFACE:${kdl_parser_INCLUDE_DIRS}> 
                       $<INSTALL_INTERFACE:${include_dest}> 
                       $<INSTALL_INTERFACE:${lib_dest}>
                       ) 

# Install                       
set(header_path "${${PROJECT_NAME}_SOURCE_DIR}/include/rct")
file(GLOB_RECURSE header "${header_path}/*.h")

install(FILES ${header} DESTINATION "${include_dest}")
install(FILES rct_lib-config.cmake DESTINATION ${lib_dest_generic})
install(TARGETS rct_lib EXPORT rct_lib-export DESTINATION "${lib_dest}")
install(EXPORT rct_lib-export DESTINATION "${lib_dest}")

# set(rct_lib_DIR "/usr/local/lib/ros_control_toolbox")
# find_package(rct_lib CONFIG REQUIRED)
