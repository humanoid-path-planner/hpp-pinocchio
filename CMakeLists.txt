# Copyright 2016 CNRS-LAAS
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

CMAKE_MINIMUM_REQUIRED(VERSION 0.1)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/ros.cmake)

SET(PROJECT_NAME hpp-pinocchio)
SET(PROJECT_DESCRIPTION "Wrapping of the kinematic/dynamic chain Pinocchio for HPP.")
SET(PROJECT_URL "No webpage yet.")

SETUP_PROJECT()

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
ADD_REQUIRED_DEPENDENCY("assimp")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-fcl >= 0.4.2")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.1")
ADD_OPTIONAL_DEPENDENCY("hpp-model >= 3")
ADD_OPTIONAL_DEPENDENCY("hpp-model-urdf >= 3")
ADD_OPTIONAL_DEPENDENCY("romeo_description >= 0.2")

ADD_DEFINITIONS(-DWITH_URDFDOM)
ADD_DEFINITIONS(-DWITH_HPP_FCL)


# Set header files.
SET(${PROJECT_NAME}_HEADERS
  include/hpp/pinocchio/fwd.hh
  include/hpp/pinocchio/device.hh
  include/hpp/pinocchio/humanoid-robot.hh
  include/hpp/pinocchio/joint.hh
  include/hpp/pinocchio/body.hh
  include/hpp/pinocchio/fake-container.hh
  include/hpp/pinocchio/device-object-vector.hh
  include/hpp/pinocchio/gripper.hh
  include/hpp/pinocchio/configuration.hh
  include/hpp/pinocchio/collision-object.hh
  include/hpp/pinocchio/extra-config-space.hh
  include/hpp/pinocchio/center-of-mass-computation.hh

  include/hpp/pinocchio/urdf/util.hh
  )

IF(HPP_MODEL_FOUND)
  LIST(APPEND ${PROJECT_NAME}_HEADERS
    include/hpp/pinocchio/hpp-model/conversions.hh
    include/hpp/pinocchio/hpp-model/model-loader.hh
    )
ENDIF(HPP_MODEL_FOUND)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
