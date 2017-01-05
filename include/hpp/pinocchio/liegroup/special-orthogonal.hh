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
        template <class ConfigL_t, class ConfigR_t>
          static double squaredDistance(
              const Eigen::MatrixBase<ConfigL_t> & q0,
              const Eigen::MatrixBase<ConfigR_t> & q1)
          {
            return se3::SpecialOrthogonalOperation<N>::squaredDistance(q0, q1);
          }

        template <class ConfigL_t, class ConfigR_t>
        static double squaredDistance(
            const Eigen::MatrixBase<ConfigL_t> & q0,
            const Eigen::MatrixBase<ConfigR_t> & q1,
            const typename ConfigL_t::Scalar& w)
        {
          return w * squaredDistance(q0, q1);
        }
      };
    } // namespace liegroup
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_LIEGROUP_SPECIAL_ORTHOGONAL_OPERATION_HH
