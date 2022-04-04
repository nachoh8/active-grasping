cmake_minimum_required(VERSION 3.0.2)

project(PyGrasp)

set(name "pygrasp")

find_package(SWIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Simox REQUIRED)
find_package(PythonLibs 2.7 REQUIRED) # 3.6 or 2.7

## INCLUDE ##
include(${SWIG_USE_FILE})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Simox_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_PATH})

include_directories("../include")

## SET CMAKE FLAGS ##
set(CMAKE_C_COMPILER "gcc-8")
set(CMAKE_CXX_COMPILER "g++-8")
set(CMAKE_CXX_STANDARD_DEFAULT)
set(CMAKE_SWIG_FLAGS -c++) # -Wall
add_definitions(-std=c++2a)

## LINK ##
set_source_files_properties(pygrasp.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(pygrasp.i PROPERTIES SWIG_FLAGS "-includeall")

set(Grasp_LIB "/home/nacho/ActiveGrasping/active-grasping-bo/Grasp/build/libGrasp.so")

swig_add_library(${name} LANGUAGE python SOURCES pygrasp.i)
swig_link_libraries(${name} ${PYTHON_LIBRARIES} ${Simox_LIBRARIES} ${Grasp_LIB})
