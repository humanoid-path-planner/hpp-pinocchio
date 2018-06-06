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

#ifndef HPP_PINOCCHIO_LIEGROUP_SPECIAL_ORTHOGONAL_OPERATION_HH
#define HPP_PINOCCHIO_LIEGROUP_SPECIAL_ORTHOGONAL_OPERATION_HH

#include <pinocchio/multibody/liegroup/special-orthogonal.hpp>

namespace hpp {
  namespace pinocchio {
    namespace liegroup {
      template<int N>
        struct SpecialOrthogonalOperation : public se3::SpecialOrthogonalOperation<N>
      {
        typedef se3::SpecialOrthogonalOperation<N> Base;
        enum {
          BoundSize = 0,
          NR = Base::NV,
          NT = 0
        };

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
          return w * squaredDistance(q0, q1);
        }

        template <class ConfigIn_t, class ConfigOut_t>
        static void setBound(
            const Eigen::MatrixBase<ConfigIn_t > & bound,
            const Eigen::MatrixBase<ConfigOut_t> & out)
        {
          if (bound.size() == 0) return;
          if (bound.size() != Base::NQ) {
            HPP_THROW(std::invalid_argument, "Expected vector of size 0 or "
                << Base::NQ << ", got size " << bound.size());
          }
          const_cast<Eigen::MatrixBase<ConfigOut_t>&>(out).head(bound.size()) = bound;
        }

        template <class JacobianIn_t, class JacobianOut_t>
        static void getRotationSubJacobian(
            const Eigen::MatrixBase<JacobianIn_t > & Jin,
            const Eigen::MatrixBase<JacobianOut_t> & Jout)
        {
          const_cast<Eigen::MatrixBase<JacobianOut_t>&> (Jout) = Jin;
        }

        template <class ConfigIn_t>
        static bool isNormalized(const Eigen::MatrixBase<ConfigIn_t > & q, const value_type& eps)
        {
          EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigIn_t , Base::ConfigVector_t);
          return (std::abs(q.norm() - 1) < eps );
        }
      };
    } // namespace liegroup
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_LIEGROUP_SPECIAL_ORTHOGONAL_OPERATION_HH
