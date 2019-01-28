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

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/ros.cmake)
INCLUDE(cmake/apple.cmake)
INCLUDE(cmake/test.cmake)

SET(PROJECT_NAME hpp-pinocchio)
SET(PROJECT_DESCRIPTION "Wrapping of the kinematic/dynamic chain Pinocchio for HPP.")
SET(PROJECT_URL "https://github.com/humanoid-path-planner/hpp-pinocchio")
OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" ON)

# Handle APPLE Cmake policy
IF(APPLE)
  APPLY_DEFAULT_APPLE_CONFIGURATION()
ENDIF(APPLE)

SETUP_HPP_PROJECT()

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
# Boost.Test is used by the test suite.
SEARCH_FOR_BOOST()

# Search for dependecies.
ADD_REQUIRED_DEPENDENCY("hpp-util >= 3.2")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2.4")

ADD_OPTIONAL_DEPENDENCY("romeo_description >= 0.2")
ADD_OPTIONAL_DEPENDENCY("hpp-environments")

# Set header files.
SET(${PROJECT_NAME}_HEADERS
  include/hpp/pinocchio/fwd.hh
  include/hpp/pinocchio/device.hh
  include/hpp/pinocchio/device-data.hh
  include/hpp/pinocchio/device-sync.hh
  include/hpp/pinocchio/humanoid-robot.hh
  include/hpp/pinocchio/joint.hh
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

  include/hpp/pinocchio/liegroup.hh
  include/hpp/pinocchio/liegroup-element.hh
  include/hpp/pinocchio/liegroup-space.hh
  include/hpp/pinocchio/liegroup/vector-space.hh
  include/hpp/pinocchio/liegroup/cartesian-product.hh
  include/hpp/pinocchio/liegroup/special-euclidean.hh
  include/hpp/pinocchio/liegroup/special-orthogonal.hh

  include/hpp/pinocchio/urdf/util.hh
  )

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SETUP_HPP_PROJECT_FINALIZE()
