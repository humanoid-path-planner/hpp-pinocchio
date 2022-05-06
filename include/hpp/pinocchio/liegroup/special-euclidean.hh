// Copyright (c) 2018, Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_LIEGROUP_SPECIAL_EUCLIDEAN_OPERATION_HH
#define HPP_PINOCCHIO_LIEGROUP_SPECIAL_EUCLIDEAN_OPERATION_HH

#include <pinocchio/multibody/liegroup/special-euclidean.hpp>

namespace hpp {
namespace pinocchio {
namespace liegroup {
template <int N>
struct SpecialEuclideanOperation
    : public ::pinocchio::SpecialEuclideanOperationTpl<N, value_type> {
  typedef ::pinocchio::SpecialEuclideanOperationTpl<N, value_type> Base;
  enum { NT = N, NR = Base::NV - N, BoundSize = NT };

  template <class ConfigL_t, class ConfigR_t>
  double squaredDistance(const Eigen::MatrixBase<ConfigL_t>& q0,
                         const Eigen::MatrixBase<ConfigR_t>& q1) {
    return Base::squaredDistance(q0, q1);
  }

  // Intentionally not implemented as it does not make sense.
  /*
  template <class ConfigL_t, class ConfigR_t>
  static double squaredDistance(
      const Eigen::MatrixBase<ConfigL_t> & q0,
      const Eigen::MatrixBase<ConfigR_t> & q1,
      const typename ConfigL_t::Scalar& w)
  {
    typedef liegroup::CartesianProductOperation<
      liegroup::VectorSpaceOperation<NT, false>,
      liegroup::SpecialOrthogonalOperation<N>
      > type;
    return type::squaredDistance (q0, q1, w);
  }
  */

  template <class ConfigIn_t, class ConfigOut_t>
  static void setBound(const Eigen::MatrixBase<ConfigIn_t>& bound,
                       const Eigen::MatrixBase<ConfigOut_t>& out) {
    if (bound.size() == Base::NQ || bound.size() == BoundSize) {
      const_cast<Eigen::MatrixBase<ConfigOut_t>&>(out).head(bound.size()) =
          bound;
    } else {
      HPP_THROW(std::invalid_argument, "Expected vector of size "
                                           << BoundSize << " or " << Base::NQ
                                           << ", got size " << bound.size());
    }
  }

  template <class JacobianIn_t, class JacobianOut_t>
  static void getRotationSubJacobian(
      const Eigen::MatrixBase<JacobianIn_t>& Jin,
      const Eigen::MatrixBase<JacobianOut_t>& Jout) {
    const_cast<Eigen::MatrixBase<JacobianOut_t>&>(Jout) =
        Jin.template bottomLeftCorner<3, 3>();
  }
};
}  // namespace liegroup
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_LIEGROUP_SPECIAL_EUCLIDEAN_OPERATION_HH
