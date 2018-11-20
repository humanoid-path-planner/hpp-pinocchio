// Copyright (c) 2017, CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_SRC_SUBSTRACTION_VISITOR_HH
# define HPP_PINOCCHIO_SRC_SUBSTRACTION_VISITOR_HH

namespace hpp {
  namespace pinocchio {
    namespace liegroupType {

      typedef Eigen::Quaternion <value_type> quaternion_t;

      /// Substraction visitor
      template <typename vector_type1, typename vector_type2>
      struct SubstractionVisitor : public boost::static_visitor <>
      {
        SubstractionVisitor (const vector_type1& e1, const vector_type2& e2,
            vector_t& res) :
          e1_ (e1), e2_ (e2), result (res),
          iq_(0), iv_(0)
        {}
        template <typename LiegroupType> void operator () (LiegroupType& op)
        {
          op.difference (
              e2_.template segment<LiegroupType::NQ>(iq_, op.nq()),
              e1_.template segment<LiegroupType::NQ>(iq_, op.nq()),
              result.segment<LiegroupType::NV>(iv_, op.nv()));

          iq_ += op.nq();
          iv_ += op.nv();
        }
        const vector_type1& e1_;
        const vector_type2& e2_;
        vector_t& result;
        size_type iq_, iv_;
      }; // struct SubstractionVisitor
    } // namespace liegroupType
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SRC_SUBSTRACTION_VISITOR_HH
