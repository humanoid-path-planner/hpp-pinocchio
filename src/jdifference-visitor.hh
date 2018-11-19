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

#ifndef HPP_PINOCCHIO_SRC_JDIFFERENCE_VISITOR_HH
# define HPP_PINOCCHIO_SRC_JDIFFERENCE_VISITOR_HH

# include <hpp/pinocchio/liegroup/vector-space.hh>

namespace hpp {
  namespace pinocchio {
    using ::pinocchio::ArgumentPosition;
    using ::pinocchio::ARG0;
    using ::pinocchio::ARG1;

    namespace liegroupType {
      template <ArgumentPosition arg, DerivativeProduct side>
      struct dDifferenceVisitor : public boost::static_visitor <>
      {
        dDifferenceVisitor (vectorIn_t& q0, vectorIn_t& q1, matrixOut_t& J)
          : q0_ (q0)
          , q1_ (q1)
          , J_ (J)
          , iq_ (0)
          , iv_ (0)
        {}

        template <typename LgT> void operator () (const LgT& lg)
        {
          typename LgT::JacobianMatrix_t Jint (lg.nv(), lg.nv());

          lg.template dDifference<arg> (
              q0_.segment<LgT::NQ>(iq_, lg.nq()),
              q1_.segment<LgT::NQ>(iq_, lg.nq()),
              Jint);
          if (side == DerivativeTimesInput)
            J_.middleRows<LgT::NV> (iv_, lg.nv()).applyOnTheLeft (Jint);
          else
            J_.middleCols<LgT::NV> (iv_, lg.nv()).applyOnTheRight (Jint);
          iq_ += lg.nq();
          iv_ += lg.nv();
        }

        template <int N, bool rot>
        void operator () (const liegroup::VectorSpaceOperation<N,rot>& lg)
        {
          if (arg == ARG0) {
            if (side == DerivativeTimesInput) J_.middleRows<N> (iv_, lg.nv()) *= -1;
            else                              J_.middleCols<N> (iv_, lg.nv()) *= -1;
          }
          iq_ += lg.nq();
          iv_ += lg.nv();
        }
        vectorIn_t & q0_, q1_;
        matrixOut_t& J_;
        size_type iq_, iv_;
      }; // struct dDifferenceVisitor
    } // namespace liegroupType
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SRC_JDIFFERENCE_VISITOR_HH
