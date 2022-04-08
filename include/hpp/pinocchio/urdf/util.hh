// Copyright (C) 2016 by Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

/// \brief Utility functions.

#ifndef HPP_PINOCCHIO_URDF_UTIL
#define HPP_PINOCCHIO_URDF_UTIL

#include <hpp/pinocchio/deprecated.hh>
#include <hpp/pinocchio/fwd.hh>
#include <pinocchio/spatial/se3.hpp>

namespace hpp {
namespace pinocchio {
namespace urdf {
/// Load robot model by name
///
/// \param robot Empty robot created before calling the function.
///        Users can pass an instance of a class deriving from Device.
/// \param baseFrame frame to which the joint tree is added.
/// \param prefix string to insert before all names
///               (joint, link, body names)
/// \param rootJointType type of root joint among "anchor", "freeflyer",
/// "planar",
/// \param package ros package containing the model
/// \param modelName robot model name
/// \param urdfSuffix suffix for urdf file
/// \param srdfSuffix suffix for srdf file
/// \param bMr position of the root joint of the new model in the base frame.
/// \note This function reads the following files:
/// \li \c package://${package}/urdf/${modelName}${urdfSuffix}.urdf
/// \li \c package://${package}/srdf/${modelName}${srdfSuffix}.srdf
void loadRobotModel(const DevicePtr_t& robot, const FrameIndex& baseFrame,
                    const std::string& prefix, const std::string& rootJointType,
                    const std::string& package, const std::string& modelName,
                    const std::string& urdfSuffix,
                    const std::string& srdfSuffix,
                    const SE3& bMr = SE3::Identity());
void loadRobotModel(const DevicePtr_t& robot, const std::string& rootJointType,
                    const std::string& package, const std::string& modelName,
                    const std::string& urdfSuffix,
                    const std::string& srdfSuffix);

/// Set the special joints of the robot (chest, waist...)
/// and initialize the gaze origin and direction
void setupHumanoidRobot(const HumanoidRobotPtr_t& robot,
                        const std::string& prefix = "");

/// Load only urdf model file
///
/// \param robot Empty robot created before calling the function.
///        Users can pass an instance of a class deriving from Device.
/// \param baseFrame frame to which the joint tree is added.
/// \param prefix string to insert before all names
///               (joint, link, body names)
/// \param rootJointType type of root joint among "anchor", "freeflyer",
/// "planar",
/// \param package ros package containing the model
/// \param filename name of the file containing the model.

/// \note This function reads the following file:
/// \li
/// package://${package}/urdf/${filename}.urdf
void loadUrdfModel(const DevicePtr_t& robot, const FrameIndex& baseFrame,
                   const std::string& prefix, const std::string& rootJointType,
                   const std::string& package, const std::string& filename,
                   const SE3& bMr = SE3::Identity());
void loadUrdfModel(const DevicePtr_t& robot, const std::string& rootJointType,
                   const std::string& package, const std::string& filename);

/// This is the base function which is called by the other function.
/// It reads a URDF file, and optionnally a SRDF file, and build the
/// robot.
/// \param srdfPath if empty, do not try to parse SRDF.
void loadModel(const DevicePtr_t& robot, const FrameIndex& baseFrame,
               const std::string& prefix, const std::string& rootType,
               const std::string& urdfPath, const std::string& srdfPath,
               const SE3& bMr = SE3::Identity());

/// Read URDF and, optionnally SRDF, as XML string.
/// \param srdfString if empty, do not try to parse SRDF.
void loadModelFromString(const DevicePtr_t& robot, const FrameIndex& baseFrame,
                         const std::string& prefix, const std::string& rootType,
                         const std::string& urdfString,
                         const std::string& srdfString,
                         const SE3& bMr = SE3::Identity());

/// Read SRDF file
void loadSRDFModel(const DevicePtr_t& robot, std::string prefix,
                   const std::string& srdfPath);

/// Read SRDF string.
void loadSRDFModelFromString(const DevicePtr_t& robot, std::string prefix,
                             const std::string& srdfString);
}  // end of namespace urdf.
}  // end of namespace pinocchio.
}  // end of namespace hpp.

#endif  // HPP_PINOCCHIO_URDF_PARSER
