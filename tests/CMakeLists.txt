# Copyright 2016 CNRS-LAAS
#
# Author: Joseph Mirabel, Nicolas Mansard from Antonio El Khoury
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# 1. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

# Make Boost.Test generates the main function in test cases.
add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Specify the tests data files directory.
add_definitions(-DTEST_DIRECTORY="${CMAKE_SOURCE_DIR}/tests/data")

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against Boost and add
# it to the test suite.
#
macro(ADD_TESTCASE NAME)
  add_unit_test(${NAME} ${NAME}.cc)
  target_link_libraries(${NAME} ${PROJECT_NAME} ${Boost_LIBRARIES})
endmacro(ADD_TESTCASE)

# --- LIST OF TESTS ------------------------------------------------------------
# --- LIST OF TESTS ------------------------------------------------------------
# --- LIST OF TESTS ------------------------------------------------------------

if(ROMEO_DESCRIPTION_FOUND)
  add_testcase(frame)
endif(ROMEO_DESCRIPTION_FOUND)

add_testcase(tconfiguration)
add_testcase(device)
add_testcase(urdf)
add_testcase(liegroup-element)
add_testcase(print)
add_testcase(joint-bounds)
add_unit_test(serialization serialization.cc)
target_link_libraries(
  serialization PRIVATE hpp-util::hpp-util pinocchio::pinocchio Boost::thread
                        ${Boost_LIBRARIES})

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  add_testcase(device-sync)
endif()
add_testcase(pool)
