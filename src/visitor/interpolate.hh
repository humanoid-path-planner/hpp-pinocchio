// Copyright (c) 2020, CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_SRC_VISITOR_INTERPOLATE_HH
# define HPP_PINOCCHIO_SRC_VISITOR_INTERPOLATE_HH

namespace hpp {
  namespace pinocchio {
    namespace liegroupType {
    namespace visitor {
      /// Interpolate
      struct Interpolate : public boost::static_visitor <>
      {
        Interpolate (const vectorIn_t& e1, const vectorIn_t& e2,
            value_type u,
            vectorOut_t& er) :
          e1_ (e1), e2_ (e2), u_ (u), er_ (er),
          iq_(0), iv_(0)
        {}
        template <typename LiegroupType> void operator () (LiegroupType& op)
        {
          op.interpolate (
              e1_.segment<LiegroupType::NQ>(iq_, op.nq()),
              e2_.segment<LiegroupType::NQ>(iq_, op.nq()),
              u_,
              er_.segment<LiegroupType::NQ>(iq_, op.nq()));

          iq_ += op.nq();
          iv_ += op.nv();
        }
        const vectorIn_t& e1_;
        const vectorIn_t& e2_;
        value_type u_;
        vectorOut_t& er_;
        size_type iq_, iv_;
      }; // struct SubstractionVisitor
    } // namespace visitor
    } // namespace liegroupType
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SRC_VISITOR_INTERPOLATE_HH
