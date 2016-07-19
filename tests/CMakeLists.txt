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

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/)

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
    ADD_EXECUTABLE(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cc)
  ELSE()
    ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  ENDIF()
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  # Link against Boost and project library.
  TARGET_LINK_LIBRARIES(${NAME}
    ${Boost_LIBRARIES}
    ${PROJECT_NAME})
  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-model-urdf)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} pinocchio)
ENDMACRO(ADD_TESTCASE)

MACRO(ADD_TESTCASE_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

# --- LIST OF TESTS ------------------------------------------------------------
# --- LIST OF TESTS ------------------------------------------------------------
# --- LIST OF TESTS ------------------------------------------------------------
IF(ROMEO_DESCRIPTION_FOUND)
  MESSAGE(STATUS "=== Tests with romeo" ${ROMEO_DESCRIPTION_DATAROOTDIR})
  ADD_TESTCASE(tfakecontainer FALSE)

  ADD_TESTCASE(tdevice FALSE)
  ADD_TESTCASE_CFLAGS(tdevice 
    '-DROMEO_MODEL_DIR=\\\"${ROMEO_DESCRIPTION_DATAROOTDIR}\\\"')

  ADD_TESTCASE(tjoint FALSE)
  ADD_TESTCASE_CFLAGS(tjoint
    '-DROMEO_MODEL_DIR=\\\"${ROMEO_DESCRIPTION_DATAROOTDIR}\\\"')

  ADD_TESTCASE(tbody FALSE)
  ADD_TESTCASE_CFLAGS(tbody
    '-DROMEO_MODEL_DIR=\\\"${ROMEO_DESCRIPTION_DATAROOTDIR}\\\"')

ENDIF(ROMEO_DESCRIPTION_FOUND)
