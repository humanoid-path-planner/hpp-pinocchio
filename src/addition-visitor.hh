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

#ifndef HPP_PINOCCHIO_SRC_ADDITION_VISITOR_HH
# define HPP_PINOCCHIO_SRC_ADDITION_VISITOR_HH

# include <Eigen/Geometry>

namespace hpp {
  namespace pinocchio {
    namespace liegroupType {

      typedef Eigen::Quaternion <value_type> quaternion_t;

      /// Addition visitor
      struct AdditionVisitor : public boost::static_visitor <>
      {
        AdditionVisitor (vectorOut_t e, vectorIn_t v) :
          e_ (e), v_ (v)
        {
        }
        template <typename LiegroupType> void operator () (LiegroupType& op)
        {
          op.integrate_impl (e_, v_, e_);
        }
        vectorOut_t e_;
        vectorIn_t v_;
      }; // struct AdditionVisitor

    } // namespace liegroupType
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SRC_ADDITION_VISITOR_HH
