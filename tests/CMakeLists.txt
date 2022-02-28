# Copyright 2016 CNRS-LAAS
#
# Author: Joseph Mirabel, Nicolas Mansard from Antonio El Khoury
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

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
ADD_UNIT_TEST(serialization serialization.cc)
target_link_libraries(serialization PRIVATE
  hpp-util::hpp-util pinocchio::pinocchio Boost::thread
  ${Boost_LIBRARIES})

FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ADD_TESTCASE(device-sync)
ENDIF()
ADD_TESTCASE(pool)
