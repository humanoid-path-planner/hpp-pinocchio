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

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

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
MACRO(ADD_TESTCASE NAME GENERATED)
  IF (${GENERATED} STREQUAL TRUE)
    ADD_UNIT_TEST(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cc)
  ELSE()
    ADD_UNIT_TEST(${NAME} ${NAME}.cc)
  ENDIF()

  # Link against Boost and project library.
  TARGET_LINK_LIBRARIES(${NAME}
    ${Boost_LIBRARIES}
    ${PROJECT_NAME})
  PKG_CONFIG_USE_DEPENDENCY(${NAME} assimp)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-util)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-fcl)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} pinocchio)
ENDMACRO(ADD_TESTCASE)

MACRO(ADD_TESTCASE_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TESTCASE_CFLAGS)

# --- LIST OF TESTS ------------------------------------------------------------
# --- LIST OF TESTS ------------------------------------------------------------
# --- LIST OF TESTS ------------------------------------------------------------

IF(ROMEO_DESCRIPTION_FOUND)
  ADD_TESTCASE(tconfiguration FALSE)
  ADD_TESTCASE_CFLAGS(tconfiguration
    '-DROMEO_MODEL_DIR=\\\"${ROMEO_DESCRIPTION_DATAROOTDIR}\\\"')

  ADD_TESTCASE(frame FALSE)

  ADD_TESTCASE(device FALSE)
ENDIF(ROMEO_DESCRIPTION_FOUND)

ADD_TESTCASE(liegroup-element FALSE)
ADD_TESTCASE(print FALSE)
