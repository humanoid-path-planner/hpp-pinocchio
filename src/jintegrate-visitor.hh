// Copyright (c) 2018, CNRS
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

#ifndef HPP_PINOCCHIO_SRC_JINTEGRATE_VISITOR_HH
# define HPP_PINOCCHIO_SRC_JINTEGRATE_VISITOR_HH

# include <hpp/pinocchio/liegroup/vector-space.hh>

namespace hpp {
  namespace pinocchio {
    namespace liegroupType {
      struct JintegrateVisitor : public boost::static_visitor <>
      {
        JintegrateVisitor (vectorIn_t& v, matrixOut_t& J, size_type& row)
          : v_ (v), J_ (J), row_ (row) {}

        template <typename LgT> void operator () (const LgT& lg)
        {
          typename LgT::JacobianMatrix_t Jint (lg.nv(), lg.nv());
          typename LgT::JacobianMatrix_t Jv (lg.nv(), lg.nv());
          LiegroupElement q(lg.neutral());
          lg.Jintegrate (q.vector(), v_.segment<LgT::NV>(row_, lg.nv()), Jint, Jv);
          J_.middleRows<LgT::NV> (row_, lg.nv()).applyOnTheLeft (Jint);
          row_ += lg.nv();
        }

        template <int N, bool rot>
        void operator () (const liegroup::VectorSpaceOperation<N,rot>& lg)
        {
          row_ += lg.nv();
        }
        vectorIn_t& v_;
        matrixOut_t& J_;
        size_type& row_;
      }; // struct JintegrateVisitor
    } // namespace liegroupType
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SRC_JINTEGRATE_VISITOR_HH
