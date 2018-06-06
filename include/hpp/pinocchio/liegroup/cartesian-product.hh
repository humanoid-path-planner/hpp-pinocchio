// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-pinocchio.
// hpp-pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-pinocchio. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_PINOCCHIO_LIEGROUP_CARTESIAN_PRODUCT_OPERATION_HH
#define HPP_PINOCCHIO_LIEGROUP_CARTESIAN_PRODUCT_OPERATION_HH

#include <pinocchio/multibody/liegroup/cartesian-product.hpp>

#include <hpp/util/exception-factory.hh>

namespace hpp {
  namespace pinocchio {
    namespace liegroup {
      template<typename LieGroup1, typename LieGroup2>
        struct CartesianProductOperation : public se3::CartesianProductOperation<LieGroup1, LieGroup2>
      {
        enum {
          BoundSize = LieGroup1::BoundSize + LieGroup2::BoundSize,
          NR = LieGroup1::NR + LieGroup2::NR,
          NT = LieGroup1::NT + LieGroup2::NT
        };

        typedef se3::CartesianProductOperation<LieGroup1, LieGroup2> Base;

        template <class ConfigL_t, class ConfigR_t>
          double squaredDistance(
              const Eigen::MatrixBase<ConfigL_t> & q0,
              const Eigen::MatrixBase<ConfigR_t> & q1)
          {
            return Base::squaredDistance(q0, q1);
          }

        template <class ConfigL_t, class ConfigR_t>
          double squaredDistance(
              const Eigen::MatrixBase<ConfigL_t> & q0,
              const Eigen::MatrixBase<ConfigR_t> & q1,
              const typename ConfigL_t::Scalar& w)
          {
            return LieGroup1().squaredDistance(q0.template head<LieGroup1::NQ>(), q1.template head<LieGroup1::NQ>(), w)
              +    LieGroup2().squaredDistance(q0.template tail<LieGroup2::NQ>(), q1.template tail<LieGroup2::NQ>(), w);
          }

        template <class ConfigIn_t, class ConfigOut_t>
        static void setBound(
            const Eigen::MatrixBase<ConfigIn_t > & bound,
            const Eigen::MatrixBase<ConfigOut_t> & out)
        {
          EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigOut_t, Base::ConfigVector_t)
          ConfigOut_t& oout = const_cast<Eigen::MatrixBase<ConfigOut_t>&> (out).derived();
          if (bound.size() == BoundSize) {
            if (LieGroup1::BoundSize > 0)
              LieGroup1::setBound(bound.template head<LieGroup1::BoundSize>(),
                                  oout. template head<LieGroup1::NQ       >());
            if (LieGroup2::BoundSize > 0)
              LieGroup2::setBound(bound.template tail<LieGroup2::BoundSize>(),
                                  oout. template tail<LieGroup2::NQ       >());
          } else if (bound.size() == Base::NQ) {
            LieGroup1::setBound(bound.template head<LieGroup1::NQ>(),
                                oout. template head<LieGroup1::NQ>());
            LieGroup2::setBound(bound.template tail<LieGroup2::NQ>(),
                                oout. template tail<LieGroup2::NQ>());
          } else {
            HPP_THROW(std::invalid_argument,
                "Expected vector of size " << BoundSize << " or " << Base::NQ
                << ", got size " << bound.size());
          }
        }

        template <class JacobianIn_t, class JacobianOut_t>
        static void getRotationSubJacobian(
            const Eigen::MatrixBase<JacobianIn_t > & Jin,
            const Eigen::MatrixBase<JacobianOut_t> & Jout)
        {
          JacobianOut_t& J = const_cast<Eigen::MatrixBase<JacobianOut_t>&> (Jout).derived();
          if (LieGroup1::NR > 0)
            LieGroup1::getRotationSubJacobian(Jin.template leftCols<LieGroup1::NV>(),
                                              J  .template leftCols<LieGroup1::NR>());
          if (LieGroup2::NR > 0)
            LieGroup2::getRotationSubJacobian(Jin.template rightCols<LieGroup2::NV>(),
                                              J  .template rightCols<LieGroup2::NR>());
        }

        template <class ConfigIn_t>
        static bool isNormalized(const Eigen::MatrixBase<ConfigIn_t > & q, const value_type& eps)
        {
          EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigIn_t , Base::ConfigVector_t);
          return LieGroup1::isNormalized(q.template head<LieGroup1::NQ>(), eps)
            &&   LieGroup2::isNormalized(q.template tail<LieGroup2::NQ>(), eps);
        }
      }; // struct CartesianProductOperation
    } // namespace liegroup
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_LIEGROUP_CARTESIAN_PRODUCT_OPERATION_HH
