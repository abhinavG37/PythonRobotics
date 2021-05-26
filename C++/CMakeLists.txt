cmake_minimum_required(VERSION 3.16)

# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# include(CCache)

project(plotter VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE )
set( CMAKE_BUILD_TYPE Debug)
endif ()

set( CMAKE_CXX_FLAGS "-Wall -Wextra ")
set( CMAKE_CXX_FLAGS_DEBUG "-g -O0")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(A_star ${CMAKE_CURRENT_SOURCE_DIR}/src/A_Star.cpp)
find_package(OpenCV 4.0 REQUIRED)
find_package(Eigen3 3.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(A_star PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(A_star PUBLIC ${OpenCV_LIBS})

add_executable(Test ${CMAKE_CURRENT_SOURCE_DIR}/src/Test.cpp)


add_executable(KalmanBasic ${CMAKE_CURRENT_SOURCE_DIR}/src/KalmanFilterBasic.cpp)
target_link_libraries(KalmanBasic PUBLIC ${OpenCV_LIBS})


add_executable(particleFilter ${CMAKE_CURRENT_SOURCE_DIR}/src/ParticleFilter.cpp)
target_link_libraries(particleFilter PUBLIC ${OpenCV_LIBS})

