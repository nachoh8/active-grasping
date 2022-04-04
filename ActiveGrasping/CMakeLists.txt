CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT (ActiveGraspingModule)

set(name "ActiveGrasping")

## FIND LIBRARIES ##

FIND_LIBRARY(NLOPT nlopt)
FIND_LIBRARY(BAYESOPT bayesopt)
MESSAGE(STATUS "** NLOPT: ${NLOPT} **")
MESSAGE(STATUS "** BAYESOPT: ${BAYESOPT} **")

if(NOT NLOPT OR NOT BAYESOPT)
  message(FATAL_ERROR "NLOPT or BAYESOPT library not found")
endif()

## CONFIG SRCS/INCLUDES ##

include_directories(${PROJECT_SOURCE_DIR}/include)
if (NOT "$ENV{BayesOpt_DIR}" STREQUAL "")
    set(BayesOpt_DIR "$ENV{BayesOpt_DIR}" CACHE INTERNAL "Copied from environment variable")
endif()
include_directories("${BayesOpt_DIR}/include")
include_directories("${BayesOpt_DIR}/utils")

include_directories("../Grasp/include")

FILE(GLOB INCS 
  ${PROJECT_SOURCE_DIR}/include/ActiveGrasping/ActiveGraspingOpt.hpp
  ${PROJECT_SOURCE_DIR}/include/ActiveGrasping/ActiveGraspingOptParams.hpp
)
  
FILE(GLOB SRCS 
  ${PROJECT_SOURCE_DIR}/src/ActiveGraspingOpt.cpp
)

set(CMAKE_C_COMPILER "gcc-8")
set(CMAKE_CXX_COMPILER "g++-8")
set(CMAKE_CXX_STANDARD_DEFAULT)
add_definitions(-std=c++2a)
add_definitions(-fPIC)

set(Grasp_LIB_DIR "/home/nacho/ActiveGrasping/active-grasping-bo/Grasp/build/libGrasp.so")

################################## LIBRARY ##############################
ADD_LIBRARY(${name} SHARED ${SRCS} ${INCS})
target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
TARGET_LINK_LIBRARIES(${name} PRIVATE bayesopt nlopt ${Grasp_LIB_DIR})

if (NOT ActiveGrasping_DIR STREQUAL "")
  set(ActiveGrasping_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "Copied from environment variable")
  set(ActiveGrasping_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE INTERNAL "Copied from environment variable")
  set(ActiveGrasping_LIB "${CMAKE_CURRENT_SOURCE_DIR}/build/libActiveGrasping.so" CACHE INTERNAL "Copied from environment variable")
endif()
