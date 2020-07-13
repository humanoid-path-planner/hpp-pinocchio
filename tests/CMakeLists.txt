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

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Specify the tests data files directory.
ADD_DEFINITIONS(-DTEST_DIRECTORY="${CMAKE_SOURCE_DIR}/tests/data")

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(ADD_TESTCASE NAME)
  ADD_UNIT_TEST(${NAME} ${NAME}.cc)
  TARGET_LINK_LIBRARIES(${NAME} ${PROJECT_NAME} ${Boost_LIBRARIES})
ENDMACRO(ADD_TESTCASE)

# --- LIST OF TESTS ------------------------------------------------------------
# --- LIST OF TESTS ------------------------------------------------------------
# --- LIST OF TESTS ------------------------------------------------------------

IF(ROMEO_DESCRIPTION_FOUND)
  ADD_TESTCASE(frame)
ENDIF(ROMEO_DESCRIPTION_FOUND)

ADD_TESTCASE(tconfiguration)
ADD_TESTCASE(device)
ADD_TESTCASE(urdf)
ADD_TESTCASE(liegroup-element)
ADD_TESTCASE(print)
ADD_TESTCASE(joint-bounds)
ADD_TESTCASE(serialization)

FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ADD_TESTCASE(device-sync)
ENDIF()
ADD_TESTCASE(pool)
