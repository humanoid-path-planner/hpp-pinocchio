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

#ifndef HPP_PINOCCHIO_LIEGROUP_VECTOR_SPACE_OPERATION_HH
#define HPP_PINOCCHIO_LIEGROUP_VECTOR_SPACE_OPERATION_HH

#include <pinocchio/multibody/liegroup/vector-space.hpp>

#include <hpp/util/exception-factory.hh>

#include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace pinocchio {
    namespace liegroup {
      /// \cond
      namespace details {
        template <bool Test> struct assign_if {
          template <class D1, class D2> static void run(
              const Eigen::MatrixBase<D1> & /*Jin*/,
              const Eigen::MatrixBase<D2> & /*Jout*/)
          {}
        };
        template <> struct assign_if<true> {
          template <class D1, class D2> static void run(
              const Eigen::MatrixBase<D1> & Jin,
              const Eigen::MatrixBase<D2> & Jout)
          {
            const_cast<Eigen::MatrixBase<D2>&> (Jout) = Jin;
          }
        };
      }
      /// \endcond

      template<int Size, bool rot>
        struct VectorSpaceOperation : public ::pinocchio::VectorSpaceOperationTpl<Size, value_type>
      {
        typedef ::pinocchio::VectorSpaceOperationTpl<Size, value_type> Base;
        enum {
          BoundSize = Size,
          NR = (rot ? Size : 0),
          NT = (rot ? 0 : Size)
        };

        /// Constructor
        /// \param size size of the vector space: should be the equal to
        ///        template argument for static sized vector-spaces.
        VectorSpaceOperation (int size = Size) : Base (size)
        {
        }

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
          if (rot) return w * squaredDistance(q0, q1);
          else     return     squaredDistance(q0, q1);
        }

        template <class ConfigIn_t, class ConfigOut_t>
        static void setBound(
            const Eigen::MatrixBase<ConfigIn_t > & bounds,
            const Eigen::MatrixBase<ConfigOut_t> & out)
        {
          if (bounds.size() != BoundSize) {
            HPP_THROW(std::invalid_argument, "Expected vector of size " <<
                (int)BoundSize << ", got size " << bounds.size());
          }
          const_cast<Eigen::MatrixBase<ConfigOut_t>&> (out) = bounds;
        }

        template <class JacobianIn_t, class JacobianOut_t>
        static void getRotationSubJacobian(
            const Eigen::MatrixBase<JacobianIn_t > & Jin,
            const Eigen::MatrixBase<JacobianOut_t> & Jout)
        {
          details::assign_if<rot>::run(Jin, Jout);
        }

        template <class ConfigIn_t>
        static bool isNormalized(const Eigen::MatrixBase<ConfigIn_t > &, const value_type&)
        {
          return true;
        }
      };
    } // namespace liegroup
  } // namespace pinocchio
} // namespace hpp

namespace pinocchio {
  template<int Size, bool rot>
  struct traits<hpp::pinocchio::liegroup::VectorSpaceOperation<Size, rot> > :
    public traits<typename hpp::pinocchio::liegroup::VectorSpaceOperation<Size, rot>::Base>
  {};
}

#endif // HPP_PINOCCHIO_LIEGROUP_VECTOR_SPACE_OPERATION_HH
