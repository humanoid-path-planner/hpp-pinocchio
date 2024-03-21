# Copyright 2016-2018 CNRS-LAAS
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

cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR TRUE)
set(PROJECT_NAME hpp-pinocchio)
set(PROJECT_DESCRIPTION
    "Wrapping of the kinematic/dynamic chain Pinocchio for HPP.")

option(INSTALL_DOCUMENTATION "Generate and install the documentation" ON)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/hpp.cmake)
include(cmake/boost.cmake)
include(cmake/apple.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(11 REQUIRED)

# Handle APPLE Cmake policy
if(APPLE)
  apply_default_apple_configuration()
endif(APPLE)

# Activate hpp-util logging if requested
set(HPP_DEBUG
    FALSE
    CACHE BOOL "trigger hpp-util debug output")
if(HPP_DEBUG)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
endif()
set(HPP_BENCHMARK
    FALSE
    CACHE BOOL "trigger hpp-util benchmarking output")
if(HPP_BENCHMARK)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_ENABLE_BENCHMARK")
endif()

# Profiling flags when building with DEBUG
# -----------------------------------------
if(CMAKE_BUILD_TYPE MATCHES "DEBUG")
  if(CMAKE_COMPILER_IS_GNUCXX)
    set(${PROJECT_NAME}_CXXFLAGS "-ggdb3 ${${PROJECT_NAME}_CXXFLAGS}")
  endif(CMAKE_COMPILER_IS_GNUCXX)
endif(CMAKE_BUILD_TYPE MATCHES "DEBUG")

# Search for Boost.
add_project_dependency(Boost REQUIRED COMPONENTS thread)

# Search for dependecies.
add_project_dependency(hpp-util 4.10.0 REQUIRED)
add_project_dependency(pinocchio REQUIRED)
add_project_dependency(Eigen3 REQUIRED)

add_project_dependency(example-robot-data)
add_project_dependency(hpp-environments)

if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
endif()

# Set header files.
set(${PROJECT_NAME}_HEADERS
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
    include/hpp/pinocchio/urdf/util.hh)

set(${PROJECT_NAME}_SOURCES
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
    src/util.cc)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC hpp-util::hpp-util example-robot-data::example-robot-data
         pinocchio::pinocchio Boost::thread)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

pkg_config_append_libs(${PROJECT_NAME})
pkg_config_append_boost_libs(thread)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
