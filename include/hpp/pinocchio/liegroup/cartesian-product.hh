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

#ifndef HPP_PINOCCHIO_LIEGROUP_CARTESIAN_PRODUCT_OPERATION_HH
#define HPP_PINOCCHIO_LIEGROUP_CARTESIAN_PRODUCT_OPERATION_HH

#include <hpp/util/exception-factory.hh>
#include <pinocchio/multibody/liegroup/cartesian-product.hpp>

namespace hpp {
namespace pinocchio {
namespace liegroup {
template <typename LieGroup1, typename LieGroup2>
struct CartesianProductOperation
    : public ::pinocchio::CartesianProductOperation<LieGroup1, LieGroup2> {
  enum {
    BoundSize = LieGroup1::BoundSize + LieGroup2::BoundSize,
    NR = LieGroup1::NR + LieGroup2::NR,
    NT = LieGroup1::NT + LieGroup2::NT
  };

  typedef ::pinocchio::CartesianProductOperation<LieGroup1, LieGroup2> Base;

  template <class ConfigL_t, class ConfigR_t>
  double squaredDistance(const Eigen::MatrixBase<ConfigL_t>& q0,
                         const Eigen::MatrixBase<ConfigR_t>& q1) {
    return Base::squaredDistance(q0, q1);
  }

  template <class ConfigL_t, class ConfigR_t>
  double squaredDistance(const Eigen::MatrixBase<ConfigL_t>& q0,
                         const Eigen::MatrixBase<ConfigR_t>& q1,
                         const typename ConfigL_t::Scalar& w) {
    return LieGroup1().squaredDistance(q0.template head<LieGroup1::NQ>(),
                                       q1.template head<LieGroup1::NQ>(), w) +
           LieGroup2().squaredDistance(q0.template tail<LieGroup2::NQ>(),
                                       q1.template tail<LieGroup2::NQ>(), w);
  }

  template <class ConfigIn_t, class ConfigOut_t>
  static void setBound(const Eigen::MatrixBase<ConfigIn_t>& bound,
                       const Eigen::MatrixBase<ConfigOut_t>& out) {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigOut_t, Base::ConfigVector_t)
    ConfigOut_t& oout =
        const_cast<Eigen::MatrixBase<ConfigOut_t>&>(out).derived();
    if (bound.size() == BoundSize) {
      if (LieGroup1::BoundSize > 0)
        LieGroup1::setBound(bound.template head<LieGroup1::BoundSize>(),
                            oout.template head<LieGroup1::NQ>());
      if (LieGroup2::BoundSize > 0)
        LieGroup2::setBound(bound.template tail<LieGroup2::BoundSize>(),
                            oout.template tail<LieGroup2::NQ>());
    } else if (bound.size() == Base::NQ) {
      LieGroup1::setBound(bound.template head<LieGroup1::NQ>(),
                          oout.template head<LieGroup1::NQ>());
      LieGroup2::setBound(bound.template tail<LieGroup2::NQ>(),
                          oout.template tail<LieGroup2::NQ>());
    } else {
      HPP_THROW(std::invalid_argument, "Expected vector of size "
                                           << (int)BoundSize << " or "
                                           << (int)Base::NQ << ", got size "
                                           << bound.size());
    }
  }

  template <class JacobianIn_t, class JacobianOut_t>
  static void getRotationSubJacobian(
      const Eigen::MatrixBase<JacobianIn_t>& Jin,
      const Eigen::MatrixBase<JacobianOut_t>& Jout) {
    JacobianOut_t& J =
        const_cast<Eigen::MatrixBase<JacobianOut_t>&>(Jout).derived();
    if (LieGroup1::NR > 0)
      LieGroup1::getRotationSubJacobian(Jin.template leftCols<LieGroup1::NV>(),
                                        J.template leftCols<LieGroup1::NR>());
    if (LieGroup2::NR > 0)
      LieGroup2::getRotationSubJacobian(Jin.template rightCols<LieGroup2::NV>(),
                                        J.template rightCols<LieGroup2::NR>());
  }

  template <class ConfigIn_t>
  static bool isNormalized(const Eigen::MatrixBase<ConfigIn_t>& q,
                           const value_type& eps) {
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigIn_t, Base::ConfigVector_t);
    return LieGroup1::isNormalized(q.template head<LieGroup1::NQ>(), eps) &&
           LieGroup2::isNormalized(q.template tail<LieGroup2::NQ>(), eps);
  }
};  // struct CartesianProductOperation
}  // namespace liegroup
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_LIEGROUP_CARTESIAN_PRODUCT_OPERATION_HH
