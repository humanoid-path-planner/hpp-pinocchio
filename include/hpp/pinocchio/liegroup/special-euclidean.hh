// Copyright (c) 2018, Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_LIEGROUP_SPECIAL_EUCLIDEAN_OPERATION_HH
#define HPP_PINOCCHIO_LIEGROUP_SPECIAL_EUCLIDEAN_OPERATION_HH

#include <pinocchio/multibody/liegroup/special-euclidean.hpp>

namespace hpp {
  namespace pinocchio {
    namespace liegroup {
      template<int N>
        struct SpecialEuclideanOperation : public se3::SpecialEuclideanOperation<N>
      {
        typedef se3::SpecialEuclideanOperation<N> Base;
        enum {
          NT = N,
          NR = Base::NV - N,
          BoundSize = NT
        };

        template <class ConfigL_t, class ConfigR_t>
          double squaredDistance(
              const Eigen::MatrixBase<ConfigL_t> & q0,
              const Eigen::MatrixBase<ConfigR_t> & q1)
          {
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
        static void setBound(
            const Eigen::MatrixBase<ConfigIn_t > & bound,
            const Eigen::MatrixBase<ConfigOut_t> & out)
        {
          if (bound.size() == Base::NQ || bound.size() == BoundSize) {
            const_cast<Eigen::MatrixBase<ConfigOut_t>&>(out).head(bound.size()) = bound;
          } else {
            HPP_THROW(std::invalid_argument, "Expected vector of size "
                << BoundSize << " or " << Base::NQ
                << ", got size " << bound.size());
          }
        }

        template <class JacobianIn_t, class JacobianOut_t>
        static void getRotationSubJacobian(
            const Eigen::MatrixBase<JacobianIn_t > & Jin,
            const Eigen::MatrixBase<JacobianOut_t> & Jout)
        {
          const_cast<Eigen::MatrixBase<JacobianOut_t>&> (Jout) = Jin.template bottomLeftCorner<3,3>();
        }

        template <class ConfigIn_t>
        static bool isNormalized(const Eigen::MatrixBase<ConfigIn_t > & q, const value_type& eps)
        {
          EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigIn_t , Base::ConfigVector_t);
          return (std::abs(q.template tail<4>().norm() - 1) < eps );
        }
      };
    } // namespace liegroup
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_LIEGROUP_SPECIAL_EUCLIDEAN_OPERATION_HH
