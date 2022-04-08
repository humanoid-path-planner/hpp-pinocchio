//
// Copyright (c) 2016 CNRS
// Author: Joseph Mirabel from Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_HUMANOID_ROBOT_HH
#define HPP_PINOCCHIO_HUMANOID_ROBOT_HH

#include <hpp/pinocchio/config.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/fwd.hh>
#include <iostream>
#include <vector>

namespace hpp {
namespace pinocchio {
/// \brief Humanoid robot

class HPP_PINOCCHIO_DLLAPI HumanoidRobot : public Device {
 public:
  /// \name Construction, copy and destruction
  /// @{
  virtual ~HumanoidRobot();

  /// \brief Clone as a HumanoidRobot
  virtual DevicePtr_t clone() const;

  HumanoidRobotPtr_t self() const { return weakPtr_.lock(); }

  ///
  /// @}
  ///

  /// \brief Creation of a new device
  /// \return a shared pointer to the new device
  /// \param name Name of the device (is passed to CkkpDeviceComponent)
  static HumanoidRobotPtr_t create(const std::string& name);

  /// \brief Get Joint corresponding to the waist.
  JointPtr_t waist() const;

  /// Set waist joint
  void waist(const JointPtr_t& joint);

  /// \brief Get Joint corresponding to the chest.
  JointPtr_t chest() const;

  /// Set chest joint
  void chest(const JointPtr_t& joint);

  /// \brief Get Joint corresponding to the left wrist.
  JointPtr_t leftWrist() const;

  /// Set left wrist
  void leftWrist(const JointPtr_t& joint);

  /// \brief Get Joint corresponding to the right wrist.
  JointPtr_t rightWrist() const;

  /// Set right wrist
  void rightWrist(const JointPtr_t& joint);

  /// \brief Get Joint corresponding to the left ankle.
  JointPtr_t leftAnkle() const;

  /// Set letf ankle
  void leftAnkle(const JointPtr_t& joint);

  /// \brief Get Joint corresponding to the right ankle.
  JointPtr_t rightAnkle() const;

  /// Set right ankle
  void rightAnkle(const JointPtr_t& joint);

  /// \brief Get gaze joint
  JointPtr_t gazeJoint() const;

  /// Set gaze joint
  void gazeJoint(const JointPtr_t& joint);

  /// Set gaze parameters
  void gaze(const vector3_t& origin, const vector3_t& dir) {
    gazeOrigin_ = origin;
    gazeDirection_ = dir;
  }

 protected:
  /// \brief Constructor
  HumanoidRobot(const std::string& name);

  HumanoidRobot(const HumanoidRobot& other);

  ///
  /// \brief Initialization.
  ///
  void init(const HumanoidRobotWkPtr_t& weakPtr);

  void initCopy(const HumanoidRobotWkPtr_t& weakPtr,
                const HumanoidRobot& other);

  /// For serialization only
  HumanoidRobot() {}

 private:
  HumanoidRobotWkPtr_t weakPtr_;
  JointPtr_t waist_;
  JointPtr_t chest_;
  JointPtr_t leftWrist_;
  JointPtr_t rightWrist_;
  JointPtr_t leftAnkle_;
  JointPtr_t rightAnkle_;
  JointPtr_t gazeJoint_;
  vector3_t gazeOrigin_;
  vector3_t gazeDirection_;

  HPP_SERIALIZABLE();
};  // class HumanoidRobot
}  // namespace pinocchio
}  // namespace hpp

BOOST_CLASS_EXPORT_KEY(hpp::pinocchio::HumanoidRobot)

#endif  // HPP_PINOCCHIO_HUMANOID_ROBOT_HH
