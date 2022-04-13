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

#ifndef HPP_PINOCCHIO_JOINT_HH
#define HPP_PINOCCHIO_JOINT_HH

#include <cstddef>
#include <hpp/pinocchio/config.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/util/serialization-fwd.hh>

namespace hpp {
namespace pinocchio {
/// Robot joint
///
/// A joint maps an input vector to a transformation of SE(3) from the
/// parent frame to the joint frame.
///
/// The input vector is provided through the configuration vector of the
/// robot the joint belongs to. The joint input vector is composed of the
/// components of the robot configuration starting at
/// Joint::rankInConfiguration.
///
/// The joint input vector represents an element of a Lie group, either
/// \li a vector space for JointTranslation, and bounded JointRotation,
/// \li the unit circle for non-bounded JointRotation joints,
/// \li an element of SO(3) for JointSO3, represented by a unit quaternion.
///
/// This class is a wrapper to pinocchio::JointModelTpl.
class HPP_PINOCCHIO_DLLAPI Joint {
 public:
  /// \name Construction and copy and destruction
  /// \{

  /// Create a new joint
  /// \param indexInJointList index in pinocchio vector of joints
  ///        (pinocchio::ModelTpl::joints)
  /// \return shared pointer to result if indexInJointList > 0, empty
  ///         shared pointer if indexInJointList == 0.
  /// \note indices of device start at 1 since 0 corresponds to "universe".
  static JointPtr_t create(DeviceWkPtr_t device, JointIndex indexInJointList);

  /// Constructor
  /// \param indexInJointList index in pinocchio vector of joints
  ///        (pinocchio::ModelTpl::joints). Should be > 0.
  /// \note indices of device start at 1 since 0 corresponds to "universe".
  Joint(DeviceWkPtr_t device, JointIndex indexInJointList);

  ~Joint() {}
  /// \}
  // -----------------------------------------------------------------------
  /// \name Name
  /// \{

  /// Get name
  const std::string& name() const;

  /// \}
  // -----------------------------------------------------------------------
  /// \name Position
  /// \{

  /// Joint transformation
  const Transform3f& currentTransformation() const {
    return currentTransformation(data());
  }

  /// Joint transformation
  const Transform3f& currentTransformation(const DeviceData& data) const;

  ///\}
  // -----------------------------------------------------------------------
  /// \name Size and rank
  ///\{

  /// Return number of degrees of freedom
  size_type numberDof() const;

  /// Return number of degrees of freedom
  size_type configSize() const;

  /// Return rank of the joint in the configuration vector
  size_type rankInConfiguration() const;

  /// Return rank of the joint in the velocity vector
  size_type rankInVelocity() const;

  ///\}
  // -----------------------------------------------------------------------
  /// \name Kinematic chain
  /// \{

  /// Get a pointer to the parent joint (if any).
  JointPtr_t parentJoint() const;

  /// Number of child joints
  std::size_t numberChildJoints() const;

  /// Get child joint
  JointPtr_t childJoint(std::size_t rank) const;

  /// Get (constant) placement of joint in parent frame, i.e.
  /// model.jointPlacement[idx]
  const Transform3f& positionInParentFrame() const;

  ///\}
  // -----------------------------------------------------------------------
  /// \name Bounds
  /// \{

  /// Set whether given degree of freedom is bounded
  /// \warning Joint always has bounds. When `bounded == false`,
  /// the bounds are `-infinity` and `infinity`.
  void isBounded(size_type rank, bool bounded);
  /// Get whether given degree of freedom is bounded
  bool isBounded(size_type rank) const;
  /// Get lower bound of given degree of freedom
  value_type lowerBound(size_type rank) const;
  /// Get upper bound of given degree of freedom
  value_type upperBound(size_type rank) const;
  /// Set lower bound of given degree of freedom
  void lowerBound(size_type rank, value_type lowerBound);
  /// Set upper bound of given degree of freedom
  void upperBound(size_type rank, value_type upperBound);
  /// Set lower bounds
  void lowerBounds(vectorIn_t lowerBounds);
  /// Set upper bounds
  void upperBounds(vectorIn_t upperBounds);

  /// Get upper bound on linear velocity of the joint frame
  /// \return coefficient \f$\lambda\f$ such that
  /// \f{equation*}
  /// \forall \mathbf{q}_{joint}\ \ \ \ \ \ \|\mathbf {v}\| \leq \lambda
  /// \|\mathbf{\dot{q}}_{joint}\| \f} where \li \f$\mathbf{q}_{joint}\f$ is any
  /// joint configuration, \li \f$\mathbf{\dot{q}}_{joint}\f$ is the joint
  /// velocity, and \li \f$\mathbf{v} = J(\mathbf{q})*\mathbf{\dot{q}} \f$ is
  /// the linear velocity of the joint frame.
  value_type upperBoundLinearVelocity() const;

  /// Get upper bound on angular velocity of the joint frame
  /// \return coefficient \f$\lambda\f$ such that
  /// \f{equation*}
  /// \forall \mathbf{q}_{joint}\ \ \ \ \ \ \|\omega\| \leq \lambda
  /// \|\mathbf{\dot{q}}_{joint}\| \f} where \li \f$\mathbf{q}_{joint}\f$ is any
  /// joint configuration, \li \f$\mathbf{\dot{q}}_{joint}\f$ is the joint
  /// velocity, and \li \f$\omega = J(\mathbf{q})*\mathbf{\dot{q}}\f$ is the
  /// angular velocity of the joint frame.
  value_type upperBoundAngularVelocity() const;

  /// Maximal distance of joint origin to parent origin
  const value_type& maximalDistanceToParent() {
    if (maximalDistanceToParent_ < 0) computeMaximalDistanceToParent();
    return maximalDistanceToParent_;
  }

  /// \}
 protected:
  /// Compute the maximal distance. \sa maximalDistanceToParent
  void computeMaximalDistanceToParent();

 public:
  // -----------------------------------------------------------------------
  /// \name Jacobian
  /// \{

  /// Get non const reference to Jacobian
  /// \param localFrame if true, compute the jacobian (6d) in the local frame,
  /// whose linear part corresponds to the velocity of the center of the frame.
  /// If false, the jacobian is expressed in the global frame and its linear
  /// part corresponds to the value of the velocity vector field at the center
  /// of the world.
  JointJacobian_t& jacobian(const bool localFrame = true) const {
    return jacobian(data(), localFrame);
  }

  /// Get reference to Jacobian
  /// \param data a DeviceData (see hpp::pinocchio::DeviceSync for details).
  /// \param localFrame if true, compute the jacobian (6d) in the local frame,
  /// whose linear part corresponds to the velocity of the center of the frame.
  /// If false, the jacobian is expressed in the global frame and its linear
  /// part corresponds to the value of the velocity vector field at the center
  /// of the world.
  JointJacobian_t& jacobian(DeviceData& data,
                            const bool localFrame = true) const;

  /// \}
  // -----------------------------------------------------------------------

  /// Access robot owning the object
  DeviceConstPtr_t robot() const { return devicePtr.lock(); }
  /// Access robot owning the object
  DevicePtr_t robot() { return devicePtr.lock(); }

  /// \name Body linked to the joint
  /// \{

  /// Get linked body
  BodyPtr_t linkedBody() const;

  /// \}

  /// Display joint
  std::ostream& display(std::ostream& os) const;

  /// Get configuration space of joint
  LiegroupSpacePtr_t configurationSpace() const;

  /// Get configuration space of joint.
  /// Use R^n x SO(n) instead of SE(n).
  LiegroupSpacePtr_t RnxSOnConfigurationSpace() const;

  /// \name Pinocchio API
  /// \{

  const JointIndex& index() const { return jointIndex; }

  /// Get the index for a given joint
  ///
  /// \return 0 if joint is NULL ("universe"), joint->index() otherwise.
  static inline size_type index(const JointConstPtr_t& joint)
  {
    return (joint ? joint->index() : 0);
  }

  const JointModel& jointModel() const;

  /// \}

  bool operator==(const Joint& other) const {
    return index() == other.index() && robot() == other.robot();
  }

  bool operator!=(const Joint& other) const { return !operator==(other); }

 protected:
  value_type maximalDistanceToParent_;
  DeviceWkPtr_t devicePtr;
  JointIndex jointIndex;
  std::vector<JointIndex> children;

  /// Store list of childrens.
  void setChildList();
  Model& model();
  const Model& model() const;
  DeviceData& data() const;

  /// Assert that the members of the struct are valid (no null pointer, etc).
  void selfAssert() const;

  friend class Device;

  Joint() {}
  HPP_SERIALIZABLE();
};  // class Joint

inline std::ostream& operator<<(std::ostream& os, const Joint& joint) {
  return joint.display(os);
}

}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_JOINT_HH
