cmake_minimum_required(VERSION 3.16)

# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# include(CCache)

project(plotter VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE )
set( CMAKE_BUILD_TYPE Debug)
endif ()

set( CMAKE_CXX_FLAGS "-Wall -Wextra ")
set( CMAKE_CXX_FLAGS_DEBUG "-g -O1")

add_executable(A_star ${CMAKE_CURRENT_SOURCE_DIR}/src/A_Star.cpp)

