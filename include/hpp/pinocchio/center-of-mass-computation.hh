// Copyright (c) 2016, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#ifndef HPP_PINOCCHIO_CENTER_OF_MASS_COMPUTATION_HH
#define HPP_PINOCCHIO_CENTER_OF_MASS_COMPUTATION_HH

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/util/serialization-fwd.hh>
#include <list>
#include <pinocchio/multibody/data.hpp>  // ::pinocchio::Data

namespace hpp {
namespace pinocchio {
/// Computation of the center of mass of a subtree of a kinematic tree
///
/// To use this class, create an instance using
/// CenterOfMassComputation::create method and call method
/// CenterOfMassComputation::add with parameter the root joint
/// of the subtree.
///
/// In most cases, the root joint of the subtree is the root joint of
/// the robot (hpp::pinocchio::Device::rootJoint ()), but in a manipulation
/// context, the kinematic tree contains several robots and objects.
/// This class enables users to compute the center of mass of only one
/// robot or object.
class CenterOfMassComputation {
 public:
  typedef std::vector<std::size_t> JointRootIndexes_t;
  /// \cond
  // This fixes an alignment issue of ::pinocchio::Data::hg
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \endcond

 public:
  /// Create instance and return shared pointer.
  ///
  /// Do not forget to call method add to specify the root joint of
  /// relevant kinematic tree.
  static CenterOfMassComputationPtr_t create(const DevicePtr_t& device);

  /// Add a subtree to the computation of the center of mass.
  ///
  /// When several subtrees are provided, method \c compute computes the
  /// center of mass of the union of the subtrees.
  void add(const JointPtr_t& rootOfSubtree);

  /// Compute the center of mass and Jacobian of the sub-trees.
  /// \param flag select which values must be computed.
  void compute(const Computation_t& flag = COMPUTE_ALL) {
    compute(robot_->d(), flag);
  }

  /// \copydoc compute(const Computation_t&)
  /// \param data where to write results.
  void compute(DeviceData& data, const Computation_t& flag);

  /// Get center of mass of the subtree.
  const vector3_t& com() const { return com(robot_->d()); }
  /// Get mass of the sub-tree.
  const value_type& mass() const { return mass(robot_->d()); }
  /// Get Jacobian of center of mass of the sub-tree.
  const ComJacobian_t& jacobian() const { return jacobian(robot_->d()); }

  /// \copydoc com
  /// \param d the data where results were written
  ///          (passed to compute(const DeviceData&, const Computation_t&)).
  static const vector3_t& com(const DeviceData& d) { return d.data_->com[0]; }
  /// \copydoc mass
  /// \param d the data where results were written
  ///          (passed to compute(const DeviceData&, const Computation_t&)).
  static const value_type& mass(const DeviceData& d) {
    return d.data_->mass[0];
  }
  /// \copydoc jacobian
  /// \param d the data where results were written
  ///          (passed to compute(const DeviceData&, const Computation_t&)).
  static const ComJacobian_t& jacobian(const DeviceData& d) {
    return d.data_->Jcom;
  }

  /// Get const reference to the vector of sub-tree roots.
  const JointRootIndexes_t& roots() const { return roots_; }

  ~CenterOfMassComputation();

 protected:
  CenterOfMassComputation(const DevicePtr_t& device);

 private:
  DevicePtr_t robot_;
  // Root of the subtrees
  JointRootIndexes_t roots_;

  CenterOfMassComputation() {}
  HPP_SERIALIZABLE();
};  // class CenterOfMassComputation
}  // namespace pinocchio
}  // namespace hpp
#endif  // HPP_PINOCCHIO_CENTER_OF_MASS_COMPUTATION_HH
