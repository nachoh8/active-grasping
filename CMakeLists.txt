cmake_minimum_required(VERSION 3.0.2)

project(Apps)

set(name "main")
add_executable(main main.cpp)

find_package(Eigen3 REQUIRED)
find_package(Simox REQUIRED)
# FIND_LIBRARY(NLOPT nlopt)
# FIND_LIBRARY(BAYESOPT bayesopt)

## INCLUDE ##
include_directories(${Simox_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

set(Grasp_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Grasp")
set(ActiveGrasping_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ActiveGrasping")

include_directories("${Grasp_DIR}/include")
include_directories("${ActiveGrasping_DIR}/include")

if (NOT "$ENV{BayesOpt_DIR}" STREQUAL "")
  set(BayesOpt_DIR "$ENV{BayesOpt_DIR}" CACHE INTERNAL "Copied from environment variable")
endif()
include_directories("${BayesOpt_DIR}/include")
include_directories("${BayesOpt_DIR}/utils")

## SET CMAKE FLAGS ##
set(CMAKE_C_COMPILER "gcc-8")
set(CMAKE_CXX_COMPILER "g++-8")
set(CMAKE_CXX_STANDARD_DEFAULT)
add_definitions(-std=c++2a)

set(Grasp_LIB "${Grasp_DIR}/build/libGrasp.so")
set(ActiveGrasping_LIB "${ActiveGrasping_DIR}/build/libActiveGrasping.so")

TARGET_LINK_LIBRARIES(${name} PRIVATE ${Grasp_LIB} ${ActiveGrasping_LIB})
