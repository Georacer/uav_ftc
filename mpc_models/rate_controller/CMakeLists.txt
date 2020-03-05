#
# Project settings
#

# Minimum required version of cmake 
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( uav_ftc CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

# activate c++ 11, taken from https://github.com/uzh-rpg/rpg_mpc/blob/master/model/CMakeLists.txt
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )

#
# Build executables
#
ADD_EXECUTABLE(        rate_controller_2_codegen rate_controller_solver_2.cpp )
TARGET_LINK_LIBRARIES( rate_controller_2_codegen ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES( rate_controller_2_codegen PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )