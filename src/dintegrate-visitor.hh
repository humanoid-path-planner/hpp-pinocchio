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

#ifndef HPP_PINOCCHIO_SRC_DINTEGRATE_VISITOR_HH
# define HPP_PINOCCHIO_SRC_DINTEGRATE_VISITOR_HH

# include <hpp/pinocchio/liegroup/vector-space.hh>

namespace hpp {
  namespace pinocchio {
    namespace liegroupType {
      struct dIntegrateVisitor_dq : public boost::static_visitor <>
      {
        dIntegrateVisitor_dq (LiegroupElement q, vectorIn_t& v, matrixOut_t& Jq, size_type& row, size_type& configRow)
          : q_(q), v_ (v), Jq_ (Jq), row_ (row), configRow_ (configRow) {}

        template <typename LgT> void operator () (const LgT& lg)
        {
          typename LgT::JacobianMatrix_t JqInt (lg.nv(), lg.nv());
          typename LgT::JacobianMatrix_t JvInt (lg.nv(), lg.nv());
          lg.Jintegrate (q_.vector().segment<LgT::NQ>(configRow_, lg.nq()), v_.segment<LgT::NV>(row_, lg.nv()), JqInt, JvInt);
          Jq_.middleRows<LgT::NV> (row_, lg.nv()).applyOnTheLeft (JqInt);
          row_ += lg.nv();
          configRow_ += lg.nq();
        }

        template <int N, bool rot>
        void operator () (const liegroup::VectorSpaceOperation<N,rot>& lg)
        {
          row_ += lg.nv();
          configRow_ += lg.nq();
        }
        LiegroupElement q_;
        vectorIn_t& v_;
        matrixOut_t& Jq_;
        size_type& row_;
        size_type& configRow_;
      }; // struct dIntegrateVisitor_dq

      struct dIntegrateVisitor_dv : public boost::static_visitor <>
      {
        dIntegrateVisitor_dv (LiegroupElement q, vectorIn_t& v, matrixOut_t& Jv, size_type& row, size_type& configRow)
          : q_(q), v_ (v), Jv_ (Jv), row_ (row), configRow_ (configRow) {}

        template <typename LgT> void operator () (const LgT& lg)
        {
          typename LgT::JacobianMatrix_t JqInt (lg.nv(), lg.nv());
          typename LgT::JacobianMatrix_t JvInt (lg.nv(), lg.nv());
          lg.Jintegrate (q_.vector().segment<LgT::NQ>(configRow_, lg.nq()), v_.segment<LgT::NV>(row_, lg.nv()), JqInt, JvInt);
          Jv_.middleRows<LgT::NV> (row_, lg.nv()).applyOnTheLeft (JvInt);
          row_ += lg.nv();
          configRow_ += lg.nq();
        }

        template <int N, bool rot>
        void operator () (const liegroup::VectorSpaceOperation<N,rot>& lg)
        {
          row_ += lg.nv();
          configRow_ += lg.nq();
        }
        LiegroupElement q_;
        vectorIn_t& v_;
        matrixOut_t& Jv_;
        size_type& row_;
        size_type& configRow_;
      }; // struct dIntegrateVisitor_dv
    } // namespace liegroupType
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SRC_JINTEGRATE_VISITOR_HH
