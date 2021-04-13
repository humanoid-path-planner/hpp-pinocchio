# Copyright 2016-2018 CNRS-LAAS
#
# Author: Joseph Mirabel, Nicolas Mansard from Antonio El Khoury
#
# This file is part of hpp-pinocchio.
# hpp-pinocchio is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hpp-pinocchio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with hpp-pinocchio.  If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(CXX_DISABLE_WERROR TRUE)
SET(PROJECT_NAME hpp-pinocchio)
SET(PROJECT_DESCRIPTION "Wrapping of the kinematic/dynamic chain Pinocchio for HPP.")

OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" ON)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/apple.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})
CHECK_MINIMAL_CXX_STANDARD(11 REQUIRED)

# Handle APPLE Cmake policy
IF(APPLE)
  APPLY_DEFAULT_APPLE_CONFIGURATION()
ENDIF(APPLE)

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()
SET (HPP_BENCHMARK FALSE CACHE BOOL "trigger hpp-util benchmarking output")
IF (HPP_BENCHMARK)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_ENABLE_BENCHMARK")
ENDIF()

# Profiling flags when building with DEBUG
#-----------------------------------------
IF(CMAKE_BUILD_TYPE MATCHES "DEBUG")
  IF(CMAKE_COMPILER_IS_GNUCXX)
    SET (${PROJECT_NAME}_CXXFLAGS "-ggdb3 ${${PROJECT_NAME}_CXXFLAGS}")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDIF(CMAKE_BUILD_TYPE MATCHES "DEBUG")

# Search for Boost.
ADD_PROJECT_DEPENDENCY(Boost REQUIRED COMPONENTS thread)

# Search for dependecies.
ADD_PROJECT_DEPENDENCY(hpp-util 4.10.0 REQUIRED)
ADD_PROJECT_DEPENDENCY(pinocchio REQUIRED)
ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED)

ADD_PROJECT_DEPENDENCY(example-robot-data)
ADD_PROJECT_DEPENDENCY(hpp-environments)

# Set header files.
SET(${PROJECT_NAME}_HEADERS
  include/hpp/pinocchio/fwd.hh
  include/hpp/pinocchio/device.hh
  include/hpp/pinocchio/device-data.hh
  include/hpp/pinocchio/device-sync.hh
  include/hpp/pinocchio/humanoid-robot.hh
  include/hpp/pinocchio/joint.hh
  include/hpp/pinocchio/joint-collection.hh
  include/hpp/pinocchio/frame.hh
  include/hpp/pinocchio/body.hh
  include/hpp/pinocchio/gripper.hh
  include/hpp/pinocchio/configuration.hh
  include/hpp/pinocchio/collision-object.hh
  include/hpp/pinocchio/extra-config-space.hh
  include/hpp/pinocchio/center-of-mass-computation.hh
  include/hpp/pinocchio/simple-device.hh
  include/hpp/pinocchio/util.hh
  include/hpp/pinocchio/pool.hh
  include/hpp/pinocchio/serialization.hh

  include/hpp/pinocchio/liegroup.hh
  include/hpp/pinocchio/liegroup-element.hh
  include/hpp/pinocchio/liegroup-space.hh
  include/hpp/pinocchio/liegroup/vector-space.hh
  include/hpp/pinocchio/liegroup/cartesian-product.hh
  include/hpp/pinocchio/liegroup/special-euclidean.hh
  include/hpp/pinocchio/liegroup/special-orthogonal.hh
  include/hpp/pinocchio/liegroup/serialization.hh

  include/hpp/pinocchio/urdf/util.hh
  )

SET(${PROJECT_NAME}_SOURCES
  src/addition-visitor.hh
  src/comparison.hh
  src/comparison.hxx
  src/device.cc
  src/device-data.cc
  src/device-sync.cc
  src/humanoid-robot.cc
  src/joint.cc
  src/frame.cc
  src/collision-object.cc
  src/body.cc
  src/gripper.cc
  src/center-of-mass-computation.cc
  src/configuration.cc
  src/simple-device.cc
  src/liegroup-element.cc
  src/liegroup-space.cc
  src/log-visitor.hh
  src/size-visitor.hh
  src/substraction-visitor.hh
  src/urdf/util.cc
  src/util.cc
  )

ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC hpp-util::hpp-util pinocchio::pinocchio Boost::thread)

INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
PKG_CONFIG_APPEND_BOOST_LIBS(thread)

INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
