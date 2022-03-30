CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT (ActiveGraspingBO)

## FIND LIBRARIES ##

FIND_LIBRARY(NLOPT nlopt)
FIND_LIBRARY(BAYESOPT bayesopt)
MESSAGE(STATUS "** NLOPT: ${NLOPT} **")
MESSAGE(STATUS "** BAYESOPT: ${BAYESOPT} **")

if(NOT NLOPT OR NOT BAYESOPT)
  message(FATAL_ERROR "NLOPT or BAYESOPT library not found")
endif()

FIND_PACKAGE(Simox REQUIRED)
MESSAGE (STATUS "** Simox version: ${Simox_VERSION}")

if(NOT (Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION))
  message(FATAL_ERROR "Simox package not found")
endif()

## CONFIG SRCS/INCLUDES ##

include_directories(${PROJECT_SOURCE_DIR}/include)
if (NOT "$ENV{BayesOpt_DIR}" STREQUAL "")
    set(BayesOpt_DIR "$ENV{BayesOpt_DIR}" CACHE INTERNAL "Copied from environment variable")
endif()
include_directories("${BayesOpt_DIR}/include")
include_directories("${BayesOpt_DIR}/utils")

FILE(GLOB INCS 
  ${PROJECT_SOURCE_DIR}/include/utils/GraspVars.hpp
  ${PROJECT_SOURCE_DIR}/include/utils/GraspResult.hpp

  ${PROJECT_SOURCE_DIR}/include/bayes/ActiveGraspingOptParams.h
  ${PROJECT_SOURCE_DIR}/include/bayes/ActiveGraspingOpt.hpp
  ${PROJECT_SOURCE_DIR}/include/bayes/GraspExecutor.hpp
  ${PROJECT_SOURCE_DIR}/include/bayes/TestGramacyExecutor.hpp
  
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspPlannerParams.hpp
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspPlanner.hpp
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspPlannerWindow.hpp
)
  
FILE(GLOB SRCS 
  ${PROJECT_SOURCE_DIR}/main.cpp
  ${PROJECT_SOURCE_DIR}/src/bayes/ActiveGraspingOpt.cpp
  ${PROJECT_SOURCE_DIR}/src/bayes/TestGramacyExecutor.cpp
  ${PROJECT_SOURCE_DIR}/src/grasp/GraspPlanner.cpp
  ${PROJECT_SOURCE_DIR}/src/grasp/GraspPlannerWindow.cpp
)

FILE(GLOB INCS_UI
  ${PROJECT_SOURCE_DIR}/include/bayes/GraspExecutor.hpp
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspPlannerParams.hpp
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspPlanner.hpp
  ${PROJECT_SOURCE_DIR}/include/grasp/GraspPlannerWindow.hpp
)

FILE(GLOB SRCS_UI
  ${PROJECT_SOURCE_DIR}/main_ui.cpp
  ${PROJECT_SOURCE_DIR}/src/grasp/GraspPlanner.cpp
  ${PROJECT_SOURCE_DIR}/src/grasp/GraspPlannerWindow.cpp
)

set(GUI_MOC_HDRS_PLANNER
    ${PROJECT_SOURCE_DIR}/include/grasp/GraspPlannerWindow.h
)

set(GUI_UIS_PLANNER
    ${PROJECT_SOURCE_DIR}/include/grasp/GraspPlanner.ui
)

## SET CMAKE FLAGS ##
set(CMAKE_C_COMPILER "gcc-8")
set(CMAKE_CXX_COMPILER "g++-8")

set(CMAKE_CXX_STANDARD_DEFAULT)
add_definitions(-std=c++2a)

## EXECUTABLE ##
SimoxQtApplication(main "${SRCS}" "${INCS}" "${GUI_MOC_HDRS_PLANNER}" "${GUI_UIS_PLANNER}")
SimoxQtApplication(mainUI "${SRCS_UI}" "${INCS_UI}" "${GUI_MOC_HDRS_PLANNER}" "${GUI_UIS_PLANNER}")

TARGET_LINK_LIBRARIES(main PRIVATE bayesopt nlopt)
TARGET_LINK_LIBRARIES(mainUI PRIVATE bayesopt nlopt)

MESSAGE( STATUS " ** END Configuration ")
