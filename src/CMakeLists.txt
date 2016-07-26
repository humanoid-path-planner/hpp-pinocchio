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

SET(LIBRARY_NAME ${PROJECT_NAME})

SET(LIBRARY_SOURCES
  device.cc
  humanoid-robot.cc
  joint.cc
  collision-object.cc
  body.cc
  device-object-vector.cc
  gripper.cc
  center-of-mass-computation.cc

  urdf/util.cc
  )

IF(HPP_MODEL_FOUND)
  LIST(APPEND LIBRARY_SOURCES hpp-model/model-loader.cc)
ENDIF(HPP_MODEL_FOUND)

#Adding headers to sources for visibility in IDE
FOREACH(infileName ${${PROJECT_NAME}_HEADERS})
    LIST(APPEND LIBRARY_SOURCES ${PROJECT_SOURCE_DIR}/${infileName})
ENDFOREACH(infileName)

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${LIBRARY_SOURCES}
  )

IF(HPP_MODEL_FOUND)
  SET_PROPERTY(TARGET ${LIBRARY_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS " -DROMEO_MODEL_DIR=\\\"${ROMEO_DESCRIPTION_DATAROOTDIR}\\\"")
ENDIF(HPP_MODEL_FOUND)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} assimp)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-fcl)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-model-urdf)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
