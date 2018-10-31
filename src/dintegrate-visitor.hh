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
      template <int arg, DerivativeProduct side>
      struct dIntegrateVisitor : public boost::static_visitor <>
      {
        dIntegrateVisitor (const vectorIn_t& q, vectorIn_t& v, matrixOut_t& Jq, size_type& row, size_type& configRow)
          : q_(q), v_ (v), Jq_ (Jq), row_ (row), configRow_ (configRow) {}

        template <typename LgT> void operator () (const LgT& lg)
        {
          // TODO add static asserts
          assert (arg == 0 || arg == 1);
          typename LgT::JacobianMatrix_t JInt (lg.nv(), lg.nv());
          if (arg == 0)
            lg.dIntegrate_dq (q_.segment<LgT::NQ>(configRow_, lg.nq()),
                              v_.segment<LgT::NV>(      row_, lg.nv()),
                              JInt);
          else
            lg.dIntegrate_dv (q_.segment<LgT::NQ>(configRow_, lg.nq()),
                              v_.segment<LgT::NV>(      row_, lg.nv()),
                              JInt);
          if   (side == DerivativeTimesInput) Jq_.middleRows<LgT::NV> (row_, lg.nv()).applyOnTheLeft  (JInt);
          else                                Jq_.middleRows<LgT::NV> (row_, lg.nv()).applyOnTheRight (JInt);
          row_ += lg.nv();
          configRow_ += lg.nq();
        }

        template <int N, bool rot>
        void operator () (const liegroup::VectorSpaceOperation<N,rot>& lg)
        {
          row_ += lg.nv();
          configRow_ += lg.nq();
        }

        const vectorIn_t& q_;
        vectorIn_t& v_;
        matrixOut_t& Jq_;
        size_type& row_;
        size_type& configRow_;
      }; // struct dIntegrateVisitor
    } // namespace liegroupType
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SRC_JINTEGRATE_VISITOR_HH
