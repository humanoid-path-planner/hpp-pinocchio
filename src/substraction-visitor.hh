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
      struct SubstractionVisitor : public boost::static_visitor <>
      {
        SubstractionVisitor (const vector_t& e1, const vector_t& e2,
                             const size_type& nv) :
          e1_ (e1), e2_ (e2), result (nv)
        {
        }
        template <typename LiegroupType> void operator () (LiegroupType& op)
        {
          op.difference_impl (e2_, e1_, result);
        }
        vector_t e1_, e2_;
        vector_t result;
      }; // struct SubstractionVisitor
    } // namespace liegroupType
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SRC_SUBSTRACTION_VISITOR_HH
