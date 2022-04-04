CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT (GraspModule)

set(name "Grasp")

FIND_PACKAGE(Simox REQUIRED)
MESSAGE (STATUS "** Simox version: ${Simox_VERSION}")
include_directories(${Simox_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


FILE(GLOB INCS 
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspVars.hpp
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspResult.hpp
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspExecutor.hpp
  ${PROJECT_SOURCE_DIR}/include/grasp/TestGramacyExecutor.hpp
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspPlannerParams.hpp
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspPlanner.hpp
)
  
FILE(GLOB SRCS 
  ${PROJECT_SOURCE_DIR}/src/TestGramacyExecutor.cpp
  ${PROJECT_SOURCE_DIR}/src/GraspPlanner.cpp
)

set(CMAKE_C_COMPILER "gcc-8")
set(CMAKE_CXX_COMPILER "g++-8")
set(CMAKE_CXX_STANDARD_DEFAULT)
add_definitions(-std=c++2a)
add_definitions(-fPIC)

################################## LIBRARY ##############################
ADD_LIBRARY(${name} SHARED ${SRCS} ${INCS})
target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
TARGET_LINK_LIBRARIES(${name} PUBLIC VirtualRobot Eigen3::Eigen GraspStudio Saba)
# TARGET_LINK_LIBRARIES(${name} PUBLIC GraspStudio Saba)
