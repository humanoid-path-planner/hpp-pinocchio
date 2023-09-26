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

#ifndef HPP_PINOCCHIO_DEVICE_SYNC_HH
#define HPP_PINOCCHIO_DEVICE_SYNC_HH

#include <hpp/pinocchio/deprecated.hh>
#include <hpp/pinocchio/device-data.hh>
#include <hpp/pinocchio/fwd.hh>

namespace hpp {
namespace pinocchio {
/// Abstract class representing a Device.
class HPP_PINOCCHIO_DLLAPI AbstractDevice {
 public:
  /// \name Access to pinocchio API
  /// \{

  /// Access to pinocchio model
  ModelConstPtr_t modelPtr() const { return model_; }
  /// Access to pinocchio model
  ModelPtr_t modelPtr() { return model_; }
  /// Access to pinocchio model
  const Model& model() const {
    assert(model_);
    return *model_;
  }
  /// Access to pinocchio model
  Model& model() {
    assert(model_);
    return *model_;
  }

  /// Access to pinocchio geomModel
  GeomModelConstPtr_t geomModelPtr() const { return geomModel_; }
  /// Access to pinocchio geomModel
  GeomModelPtr_t geomModelPtr() { return geomModel_; }
  /// Access to pinocchio geomModel
  const GeomModel& geomModel() const {
    assert(geomModel_);
    return *geomModel_;
  }
  /// Access to pinocchio geomModel
  GeomModel& geomModel() {
    assert(geomModel_);
    return *geomModel_;
  }

  /// Access to Pinocchio data/
  DataConstPtr_t dataPtr() const { return d().data_; }
  /// Access to Pinocchio data/
  DataPtr_t dataPtr() { return d().data_; }
  /// Access to Pinocchio data/
  const Data& data() const {
    assert(d().data_);
    return *d().data_;
  }
  /// Access to Pinocchio data/
  Data& data() {
    assert(d().data_);
    return *d().data_;
  }

  /// Access to Pinocchio geomData/
  GeomDataConstPtr_t geomDataPtr() const { return d().geomData_; }
  /// Access to Pinocchio geomData/
  GeomDataPtr_t geomDataPtr() { return d().geomData_; }
  /// Access to Pinocchio geomData/
  const GeomData& geomData() const {
    assert(d().geomData_);
    return *d().geomData_;
  }
  /// Access to Pinocchio geomData/
  GeomData& geomData() {
    assert(d().geomData_);
    return *d().geomData_;
  }

  /// \}
  // -----------------------------------------------------------------------
  /// \name Current state
  /// \{

  /// Get current configuration
  const Configuration_t& currentConfiguration() const {
    return d().currentConfiguration_;
  }
  /// Set current configuration
  /// \return True if the current configuration was modified and false if
  ///         the current configuration did not change.
  virtual bool currentConfiguration(ConfigurationIn_t configuration);

  /// Get current velocity
  const vector_t& currentVelocity() const { return d().currentVelocity_; }

  /// Set current velocity
  bool currentVelocity(vectorIn_t velocity);

  /// Get current acceleration
  const vector_t& currentAcceleration() const {
    return d().currentAcceleration_;
  }

  /// Set current acceleration
  bool currentAcceleration(vectorIn_t acceleration);
  /// \}
  // -----------------------------------------------------------------------
  /// \name Mass and center of mass
  /// \{

  /// Get mass of robot
  const value_type& mass() const;

  /// Get position of center of mass
  const vector3_t& positionCenterOfMass() const;

  /// Get Jacobian of center of mass with respect to configuration
  const ComJacobian_t& jacobianCenterOfMass() const;

  /// \}
  // -----------------------------------------------------------------------
  /// \name Forward kinematics
  /// \{

  /// \deprecated Use computeForwardKinematics(Computation_t) instead to select
  /// what should be computed.
  void controlComputation(const Computation_t& flag) HPP_PINOCCHIO_DEPRECATED {}
  /// \deprecated returns COMPUTE_ALL
  Computation_t computationFlag() const HPP_PINOCCHIO_DEPRECATED {
    return COMPUTE_ALL;
  }
  /// Compute forward kinematics computing everything
  void computeForwardKinematics() { computeForwardKinematics(COMPUTE_ALL); }
  /// Compute forward kinematics with custom computation flag
  /// \param flag to customise the computation. This should be a bitwise OR
  ///        between Computation_t values.
  void computeForwardKinematics(int flag) {
    d().computeForwardKinematics(modelPtr(), flag);
  }
  /// Compute frame forward kinematics
  /// \note call AbstractDevice::computeForwardKinematics.
  void computeFramesForwardKinematics() {
    d().computeFramesForwardKinematics(modelPtr());
  }
  /// Update the geometry placement to the currentConfiguration
  void updateGeometryPlacements() {
    d().updateGeometryPlacements(modelPtr(), geomModelPtr());
  }
  /// \}
  // -----------------------------------------------------------------------
 protected:
  AbstractDevice();
  AbstractDevice(const ModelPtr_t& m, const GeomModelPtr_t& gm);

  virtual DeviceData& d() = 0;
  virtual DeviceData const& d() const = 0;

  // Pinocchio objects
  ModelPtr_t model_;
  GeomModelPtr_t geomModel_;
};  // class AbstractDevice

/// A thread-safe access to a Device.
///
/// To ensure thread safety, one can do
/// \code{cpp}
/// // In some place of the code:
/// DevicePtr_t device = ...;
/// device->numberDeviceData (4);
///
/// // Acquires a lock on the device.
/// DeviceSync deviceSync (device);
/// deviceSync.currentConfiguration(q);
/// deviceSync.computeForwardKinematics(JOINT_POSITION | JACOBIAN);
///
/// JointPtr_t joint = ...;
/// joint->currentTransformation (deviceSync.d());
///
/// CollisionObjectPtr_t body = ...;
/// body->fcl          (deviceSync.d());
/// body->getTransform (deviceSync.d());
///
/// // The lock is release when deviceSync is destroyed.
/// \endcode
class HPP_PINOCCHIO_DLLAPI DeviceSync : public AbstractDevice {
 public:
  /// Constructor
  /// \param device to lock
  /// \param lock whether to acquire the lock.
  DeviceSync(const DevicePtr_t& device, bool lock = true);

  /// Destructor.
  /// The lock is released if necessary.
  virtual ~DeviceSync();

  /// Accessor to the locked DeviceData.
  /// \note this asserts that this isLocked()
  DeviceData& d() {
    assert(isLocked());
    return *d_;
  }
  /// Const accessor to the locked DeviceData.
  /// \note this asserts that this isLocked()
  DeviceData const& d() const {
    assert(isLocked());
    return *d_;
  }

  /// Lock the current DeviceData
  void lock();
  /// Check if the current DeviceData is locked
  bool isLocked() const { return d_ != NULL; }
  /// Unlock the current DeviceData
  void unlock();

 private:
  DevicePtr_t device_;
  DeviceData* d_;
};  // class DeviceSync
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_DEVICE_SYNC_HH
