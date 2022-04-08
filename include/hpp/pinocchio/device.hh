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

#ifndef HPP_PINOCCHIO_DEVICE_HH
#define HPP_PINOCCHIO_DEVICE_HH

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <hpp/pinocchio/config.hh>
#include <hpp/pinocchio/deprecated.hh>
#include <hpp/pinocchio/device-data.hh>
#include <hpp/pinocchio/device-sync.hh>
#include <hpp/pinocchio/extra-config-space.hh>
#include <hpp/pinocchio/frame.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/pinocchio/pool.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/serialization-fwd.hh>
#include <iostream>
#include <list>
#include <vector>

namespace hpp {
namespace pinocchio {

/// \brief Robot with geometric and dynamic pinocchio

/// The creation of the device is done by Device::create(const
/// std::string name).  This function returns a shared pointer
/// to the newly created object.  \sa Smart pointers
/// documentation:
/// http://www.boost.org/libs/smart_ptr/smart_ptr.htm
class HPP_PINOCCHIO_DLLAPI Device : public AbstractDevice {
  friend class Joint;
  friend class Frame;
  friend class DeviceSync;
  friend class CollisionObject;
  friend class CenterOfMassComputation;

 public:
  /// Collision pairs between bodies
  typedef std::pair<JointPtr_t, JointPtr_t> CollisionPair_t;
  typedef std::list<CollisionPair_t> CollisionPairs_t;

  // -----------------------------------------------------------------------
  /// \name Construction, copy and destruction
  /// \{
  virtual ~Device();

  /// \brief Clone as a CkwsDevice
  /// The pinocchio model is not copied (only copy the pointer).
  /// A new Pinocchio "data" is created.
  /// As the model is not copied, cloning is a non constant operation. \sa
  /// cloneConst
  virtual DevicePtr_t clone() const { return createCopy(weakPtr_.lock()); }
  /// \brief Clone as a CkwsDevice
  /// Both pinocchio objects model and data are copied.
  /// TODO: this method is not implemented yet (assert if called)
  DevicePtr_t cloneConst() const { return createCopyConst(weakPtr_.lock()); }

  DevicePtr_t self() const { return weakPtr_.lock(); }

  /// Get name of device
  const std::string& name() const { return name_; }

  /// \brief Creation of a new device
  /// \return a shared pointer to the new device
  /// \param name Name of the device
  static DevicePtr_t create(const std::string& name);

  /// \brief Copy of a device
  /// \return A shared pointer to new device.
  /// \param device Device to be copied.
  /// The pinocchio model is not copied (only copy the pointer).
  /// A new Pinocchio "data" is created.
  static DevicePtr_t createCopy(const DevicePtr_t& device);
  static DevicePtr_t createCopyConst(const DeviceConstPtr_t& device);

  /// \}
  // -----------------------------------------------------------------------
  /// \name Set pinocchio models and datas
  /// \{

  /// Set pinocchio model.
  void setModel(ModelPtr_t modelPtr) { model_ = modelPtr; }

  /// Set pinocchio geom.
  void setGeomModel(GeomModelPtr_t geomModelPtr) { geomModel_ = geomModelPtr; }

  /// Set Pinocchio data corresponding to model
  void setData(DataPtr_t dataPtr) {
    d().data_ = dataPtr;
    resizeState();
  }
  /// Create Pinocchio data from model.
  void createData();

  /// Set Pinocchio geomData corresponding to model
  void setGeomData(GeomDataPtr_t geomDataPtr) {
    d().geomData_ = geomDataPtr;
    resizeState();
  }
  /// Create Pinocchio geomData from model.
  void createGeomData();

  /// Remove some joints from the configuration space.
  virtual void removeJoints(const std::vector<std::string>& jointNames,
                            Configuration_t referenceConfig);

  /// \}
  // -----------------------------------------------------------------------
  /// \name Joints
  /// \{

  /// This struct defines a linear relation between two joint values.
  ///
  /// The configuration of \c JointConstraint::joint is expected to be \f$
  /// q_{joint} = multiplier * q_{reference} + offset\f$.
  /// where \f$q_{reference}\f$ is the configuration of
  /// JointLinearConstraint::reference joint.
  struct JointLinearConstraint {
    value_type offset, multiplier;
    JointPtr_t joint, reference;
  };

  /// Get root joint
  JointPtr_t rootJoint() const;

  /// Get root frame
  Frame rootFrame() const;

  /// Get number of joints
  /// \note "universe" is not included in the count
  size_type nbJoints() const;

  /// Access i-th joint
  /// \param i index of joint in device. This is 1 less than index of joint
  ///        in pinocchio vector of joints (pinocchio::ModelTpl::joints)
  ///        since "universe" is not included as index 0
  JointPtr_t jointAt(const size_type& i) const;

  /// Get the joint at configuration rank r
  /// \return  joint j such that j->rankInConfiguration () <=
  ///          r < j->rankInConfiguration () + j->configSize ()
  JointPtr_t getJointAtConfigRank(const size_type& r) const;

  /// Get the joint at velocity rank r
  /// \return  joint j such that j->rankInVelocity () <=
  ///          r < j->rankInVelocity () + j->numberDof ()
  JointPtr_t getJointAtVelocityRank(const size_type& r) const;

  /// Get joint by name
  /// \param name name of the joint.
  /// \throw runtime_error if device has no joint with this name
  JointPtr_t getJointByName(const std::string& name) const;

  /// Get joint by body name
  /// \throw runtime_error if device has no body with this name
  JointPtr_t getJointByBodyName(const std::string& name) const;

  /// Get frame by name
  /// \param name name of the frame.
  /// \throw runtime_error if device has no frame with this name
  Frame getFrameByName(const std::string& name) const;

  /// Size of configuration vectors
  /// Sum of joint dimensions and of extra configuration space dimension
  size_type configSize() const;

  /// Size of velocity vectors
  /// Sum of joint number of degrees of freedom and of extra configuration
  /// space dimension
  size_type numberDof() const;

  /// Returns a LiegroupSpace representing the configuration space.
  const LiegroupSpacePtr_t& configSpace() const { return configSpace_; }

  /// See Joint::RnxSOnConfigurationSpace
  const LiegroupSpacePtr_t& RnxSOnConfigSpace() const {
    return configSpaceRnxSOn_;
  }

  /// Get the neutral configuration
  Configuration_t neutralConfiguration() const;

  /// Add a joint constraint
  void addJointConstraint(JointLinearConstraint constraint);

  const std::vector<JointLinearConstraint>& jointConstraints() const {
    return jointConstraints_;
  }

  /// \}
  // -----------------------------------------------------------------------
  /// \name Extra configuration space
  /// \{

  /// Get degrees of freedom to store internal values in configurations
  ///
  /// In some applications, it is useful to store extra variables with
  /// the configuration vector. For instance, when planning motions in
  /// state space using roadmap based methods, the velocity of the robot
  /// is stored in the nodes of the roadmap.
  ExtraConfigSpace& extraConfigSpace() { return extraConfigSpace_; }

  /// Get degrees of freedom to store internal values in configurations
  ///
  /// In some applications, it is useful to store extra variables with
  /// the configuration vector. For instance, when planning motions in
  /// state space using roadmap based methods, the velocity of the robot
  /// is stored in the nodes of the roadmap.
  const ExtraConfigSpace& extraConfigSpace() const { return extraConfigSpace_; }

  /// Set dimension of extra configuration space
  virtual void setDimensionExtraConfigSpace(const size_type& dimension) {
    extraConfigSpace_.setDimension(dimension);
    resizeState();
  }

  /// \}
  // -----------------------------------------------------------------------
  /// \name Grippers
  /// \{

  /// Add a gripper to the Device
  void addGripper(const GripperPtr_t& gripper) { grippers_.push_back(gripper); }

  /// Return list of grippers of the Device
  Grippers_t& grippers() { return grippers_; }

  /// Return list of grippers of the Device
  const Grippers_t& grippers() const { return grippers_; }

  /// \}
  // -----------------------------------------------------------------------
  /// \name Collision and distance computation
  /// \{

  /// Get the static obstacles
  /// It corresponds to the geometries attached to the *universe* in
  /// pinocchio.
  BodyPtr_t obstacles() const;

  /// Number of objects
  size_type nbObjects() const;

  /// Access the i-th object.
  /// \sa Device::nbObjects() const
  CollisionObjectPtr_t objectAt(const size_type& i) const;

  /// Test collision of current configuration
  /// \param stopAtFirstCollision act as named
  /// \warning Users should call computeForwardKinematics first.
  bool collisionTest(const bool stopAtFirstCollision = true);

  /// Compute distances between pairs of objects stored in bodies
  /// \warning Users should call computeForwardKinematics first.
  void computeDistances();

  /// Get result of distance computations
  const DistanceResults_t& distanceResults() const;
  /// \}
  // -----------------------------------------------------------------------
  /// \name Multithreading
  /// See DeviceSync for how to enable thread safety.
  /// \{

  /// Set the maximum number of concurrent use of the Device.
  void numberDeviceData(const size_type& s);

  /// Get the number of DeviceData.
  size_type numberDeviceData() const;

  /// \}
  // -----------------------------------------------------------------------

  /// Print object in a stream
  virtual std::ostream& print(std::ostream& os) const;

  /// Compute an aligned bounding around the robot.
  /// The bounding box is computed as follows:
  /// - for each direct children of the universe joint:
  ///   - compute a bounding box of its center (using the bounds in translation)
  ///   - compute the maximal distance between the its center and
  ///     each bodies attach its subtree.
  ///   - sum the two.
  /// - sum all the BB obtained.
  fcl::AABB computeAABB() const;

  void controlComputation(const Computation_t& flag);

 protected:
  /// \brief Constructor
  Device(const std::string& name);

  /// \brief Initialization.
  ///
  void init(const DeviceWkPtr_t& weakPtr);
  /// \brief Initialization of of a clone device.
  ///
  void initCopy(const DeviceWkPtr_t& weakPtr, const Device& other);

  /// \brief Copy Constructor
  Device(const Device& device);

 private:
  /// Resize configuration when changing data or extra-config.
  void resizeState();

 protected:
  DeviceData d_;

  DeviceData& d() { return d_; }
  DeviceData const& d() const { return d_; }

  inline void invalidate() { d_.invalidate(); }

  std::string name_;
  // Grippers
  Grippers_t grippers_;
  LiegroupSpacePtr_t configSpace_, configSpaceRnxSOn_;
  // Extra configuration space
  ExtraConfigSpace extraConfigSpace_;
  // Joint linear constraints
  std::vector<JointLinearConstraint> jointConstraints_;
  DeviceWkPtr_t weakPtr_;

 private:
  Pool<DeviceData> datas_;

 protected:
  Device() {}

 private:
  HPP_SERIALIZABLE_SPLIT();
};  // class Device

void replaceGeometryByConvexHull(GeomModel& geomModel,
                                 const std::vector<std::string>& geometryNames);

inline std::ostream& operator<<(std::ostream& os,
                                const hpp::pinocchio::Device& device) {
  return device.print(os);
}

}  // namespace pinocchio
}  // namespace hpp

BOOST_CLASS_EXPORT_KEY(hpp::pinocchio::Device)

#endif  // HPP_PINOCCHIO_DEVICE_HH
