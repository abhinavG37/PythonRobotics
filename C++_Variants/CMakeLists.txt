cmake_minimum_required(VERSION 3.16)

# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# include(CCache)

project(plotter VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

add_executable(plotter ${CMAKE_CURRENT_SOURCE_DIR}/src/plotter.cpp)

