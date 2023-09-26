//
// Copyright (c) 2016 CNRS
// Author: NMansard from Florent Lamiraux
//
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

#ifndef HPP_PINOCCHIO_DEVICE_DATA_HH
#define HPP_PINOCCHIO_DEVICE_DATA_HH

#include <hpp/pinocchio/config.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/util/debug.hh>

namespace hpp {
namespace pinocchio {
enum Computation_t {
  JOINT_POSITION = 0x1,
  JACOBIAN = 0x2,
  VELOCITY = 0x4,
  ACCELERATION = 0x8,
  COM = 0xf,
  COMPUTE_ALL = 0Xffff
};

/// Struct containing the Device data.
/// Users normally do not need to access its attributes.
struct DeviceData {
  DeviceData();
  DeviceData(const DeviceData& other);

  inline void invalidate() {
    dataUpToDate_ = 0;
    frameUpToDate_ = false;
    geomUpToDate_ = false;
  }

  /// \param flag to customise the computation. This should be a bitwise OR
  ///        between Computation_t values.
  void computeForwardKinematics(const ModelPtr_t& m, int flag);
  void computeFramesForwardKinematics(const ModelPtr_t& m);
  void updateGeometryPlacements(const ModelPtr_t& m, const GeomModelPtr_t& gm);

  // Pinocchio objects
  DataPtr_t data_;
  GeomDataPtr_t geomData_;

  Configuration_t currentConfiguration_;
  vector_t currentVelocity_;
  vector_t currentAcceleration_;
  int dataUpToDate_;
  bool frameUpToDate_, geomUpToDate_;
  DeviceWkPtr_t devicePtr_;

  /// Temporary variable to avoid dynamic allocation
  Configuration_t modelConf_;
  /// Pool of joint jacobians
  std::vector<JointJacobian_t> jointJacobians_;
};  // struct DeviceData
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_DEVICE_DATA_HH
