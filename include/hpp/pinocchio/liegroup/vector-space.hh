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
      /// \endcond

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
        VectorSpaceOperation (int size = std::max(0,Size)) : Base (size)
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
