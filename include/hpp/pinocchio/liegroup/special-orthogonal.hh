// Copyright (c) 2017, Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_LIEGROUP_SPECIAL_ORTHOGONAL_OPERATION_HH
#define HPP_PINOCCHIO_LIEGROUP_SPECIAL_ORTHOGONAL_OPERATION_HH

#include <pinocchio/multibody/liegroup/special-orthogonal.hpp>

namespace hpp {
namespace pinocchio {
namespace liegroup {
template <int N>
struct SpecialOrthogonalOperation
    : public ::pinocchio::SpecialOrthogonalOperationTpl<N, value_type> {
  typedef ::pinocchio::SpecialOrthogonalOperationTpl<N, value_type> Base;
  enum { BoundSize = 0, NR = Base::NV, NT = 0 };

  template <class ConfigL_t, class ConfigR_t>
  double squaredDistance(const Eigen::MatrixBase<ConfigL_t>& q0,
                         const Eigen::MatrixBase<ConfigR_t>& q1) {
    return Base::squaredDistance(q0, q1);
  }

  template <class ConfigL_t, class ConfigR_t>
  double squaredDistance(const Eigen::MatrixBase<ConfigL_t>& q0,
                         const Eigen::MatrixBase<ConfigR_t>& q1,
                         const typename ConfigL_t::Scalar& w) {
    return w * squaredDistance(q0, q1);
  }

  template <class ConfigIn_t, class ConfigOut_t>
  static void setBound(const Eigen::MatrixBase<ConfigIn_t>& bound,
                       const Eigen::MatrixBase<ConfigOut_t>& out) {
    if (bound.size() == 0) return;
    if (bound.size() != Base::NQ) {
      HPP_THROW(std::invalid_argument, "Expected vector of size 0 or "
                                           << (int)Base::NQ << ", got size "
                                           << bound.size());
    }
    const_cast<Eigen::MatrixBase<ConfigOut_t>&>(out).head(bound.size()) = bound;
  }

  template <class JacobianIn_t, class JacobianOut_t>
  static void getRotationSubJacobian(
      const Eigen::MatrixBase<JacobianIn_t>& Jin,
      const Eigen::MatrixBase<JacobianOut_t>& Jout) {
    const_cast<Eigen::MatrixBase<JacobianOut_t>&>(Jout) = Jin;
  }

  template <class ConfigIn_t>
  static bool isNormalized(const Eigen::MatrixBase<ConfigIn_t>& q,
                           const value_type& eps) {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigIn_t, Base::ConfigVector_t);
    return (std::abs(q.norm() - 1) < eps);
  }
};
}  // namespace liegroup
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_LIEGROUP_SPECIAL_ORTHOGONAL_OPERATION_HH
