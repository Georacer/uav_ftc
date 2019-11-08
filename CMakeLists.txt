cmake_minimum_required(VERSION 2.8.3)
project(uav_ftc)

# activate c++ 11, taken from https://github.com/uzh-rpg/rpg_mpc/blob/master/CMakeLists.txt
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
	message_generation
  last_letter_msgs
	last_letter_lib
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Parameter.msg
  BusData.msg
  FlightEnvelopeEllipsoid.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/flight_envelope.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include include/data_manipulators
#  LIBRARIES uav_ftc
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs last_letter_lib message_runtime
 DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/data_manipulators
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	${last_letter_lib_INCLUDE_DIRS}
)

## Declare a C++ library

# The rate controller solver library
add_library(rate_controller_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  mpc_models/rate_controller/rate_controller_mpc_solver/acado_qpoases_interface.cpp
  mpc_models/rate_controller/rate_controller_mpc_solver/acado_integrator.c
  mpc_models/rate_controller/rate_controller_mpc_solver/acado_solver.c
  mpc_models/rate_controller/rate_controller_mpc_solver/acado_auxiliary_functions.c
)
target_include_directories(rate_controller_solver PUBLIC
  mpc_models/rate_controller/rate_controller_mpc_solver/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
)

# The rate controller solver library, with full state constraints, nominal
add_library(rate_controller_solver_2
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  mpc_models/rate_controller_2/rate_controller_mpc_solver_2/acado_qpoases_interface.cpp
  mpc_models/rate_controller_2/rate_controller_mpc_solver_2/acado_integrator.c
  mpc_models/rate_controller_2/rate_controller_mpc_solver_2/acado_solver.c
  mpc_models/rate_controller_2/rate_controller_mpc_solver_2/acado_auxiliary_functions.c
)
target_include_directories(rate_controller_solver_2 PUBLIC
  mpc_models/rate_controller_2/rate_controller_mpc_solver_2/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
)

# The trajectory controller solver library
add_library(trajectory_controller_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  mpc_models/trajectory_controller/trajectory_controller_mpc_solver/acado_qpoases_interface.cpp
  mpc_models/trajectory_controller/trajectory_controller_mpc_solver/acado_integrator.c
  mpc_models/trajectory_controller/trajectory_controller_mpc_solver/acado_solver.c
  mpc_models/trajectory_controller/trajectory_controller_mpc_solver/acado_auxiliary_functions.c
)
target_include_directories(trajectory_controller_solver PUBLIC
  mpc_models/trajectory_controller/trajectory_controller_mpc_solver/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
)

# The trajectory controller solver 2 library, with Va, theta constraints
add_library(trajectory_controller_solver_2
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  mpc_models/trajectory_controller_2/trajectory_controller_mpc_solver_2/acado_qpoases_interface.cpp
  mpc_models/trajectory_controller_2/trajectory_controller_mpc_solver_2/acado_integrator.c
  mpc_models/trajectory_controller_2/trajectory_controller_mpc_solver_2/acado_solver.c
  mpc_models/trajectory_controller_2/trajectory_controller_mpc_solver_2/acado_auxiliary_functions.c
)
target_include_directories(trajectory_controller_solver_2 PUBLIC
  mpc_models/trajectory_controller_2/trajectory_controller_mpc_solver_2/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
)

# The rate controller wrapper
add_library(rate_controller_wrapper
  src/mpc_wrapper.cpp
)
target_link_libraries(rate_controller_wrapper
  rate_controller_solver
)

# The rate controller 2 wrapper, with full state constraints, no fault
add_library(rate_controller_wrapper_2
  src/mpc_wrapper.cpp
)
target_link_libraries(rate_controller_wrapper_2
  rate_controller_solver_2
)

# The trajectory controller wrapper
add_library(trajectory_controller_wrapper
  src/mpc_wrapper.cpp
)
target_link_libraries(trajectory_controller_wrapper
  trajectory_controller_solver
)

# The trajectory controller 2 wrapper
add_library(trajectory_controller_wrapper_2
  src/mpc_wrapper.cpp
)
target_link_libraries(trajectory_controller_wrapper_2
  trajectory_controller_solver_2
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

# The rate controller
add_executable(rate_controller
  src/rate_controller.cpp
)
add_dependencies(rate_controller ${catkin_EXPORTED_TARGETS} )
target_link_libraries(rate_controller
  ${catkin_LIBRARIES}
  rate_controller_wrapper
)

# The rate controller_2, with full state constraints, no faults
add_executable(rate_controller_2
  src/rate_controller_2.cpp
)
add_dependencies(rate_controller_2 ${catkin_EXPORTED_TARGETS} )
target_link_libraries(rate_controller_2
  ${catkin_LIBRARIES}
  rate_controller_wrapper_2
)

# The trajectory controller
add_executable(trajectory_controller
  src/trajectory_controller.cpp
)
add_dependencies(trajectory_controller ${catkin_EXPORTED_TARGETS} )
target_link_libraries(trajectory_controller
  ${catkin_LIBRARIES}
  trajectory_controller_wrapper
)

# The trajectory controller 2
add_executable(trajectory_controller_2
  src/trajectory_controller_2.cpp
)
add_dependencies(trajectory_controller_2 ${catkin_EXPORTED_TARGETS} )
target_link_libraries(trajectory_controller_2
  ${catkin_LIBRARIES}
  trajectory_controller_wrapper_2
)

# Node reading from joystick and generating reference commands
add_executable(reference_generator
  src/reference_generator.cpp
)
add_dependencies(rate_controller ${catkin_EXPORTED_TARGETS} )
target_link_libraries(reference_generator
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
)

# Node reading from inputs sources and combinging control signals
add_executable(input_aggregator
  src/input_aggregator.cpp
)
add_dependencies(input_aggregator ${catkin_EXPORTED_TARGETS} )
target_link_libraries(input_aggregator
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
)

# Node reading from sensors and building the BusData message
add_executable(data_bus
  src/data_manipulators/data_bus.cpp
)
add_dependencies(data_bus ${catkin_EXPORTED_TARGETS} )
target_link_libraries(data_bus
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# Install scenarios alongside the executables
# TODO: This needs the install step of the catkin build command, but I'm hitting some errors there
install(DIRECTORY data/scenarios/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    # FILES_MATCHNG PATTERN "*.json"
)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uav_ftc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
