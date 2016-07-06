# Copyright 2012, 2013, 2014 CNRS-LAAS
#
# Author: Antonio El Khoury
#
# This file is part of hpp-model-urdf.
# hpp-model-urdf is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hpp-model-urdf is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with hpp-model-urdf.  If not, see <http://www.gnu.org/licenses/>.

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

SET(TEST_WITH_ROMEO OFF CACHE BOOL
  "Enable test with Romeo model: trigger dependency to romeo package.")

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
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.1")
ADD_OPTIONAL_DEPENDENCY("hpp-model >= 3")
ADD_OPTIONAL_DEPENDENCY("hpp-model-urdf >= 3")

IF (${TEST_WITH_ROMEO} STREQUAL ON)
  ADD_REQUIRED_DEPENDENCY ("romeo_description")
ENDIF()

# Set header files.
SET(${PROJECT_NAME}_HEADERS
  include/hpp/pinocchio/device.hh
  include/hpp/pinocchio/joint.hh
  )

INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
  DESTINATION include/hpp/pinocchio
  )

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
