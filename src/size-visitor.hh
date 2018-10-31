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

#ifndef HPP_PINOCCHIO_SRC_SIZE_VISITOR_HH
# define HPP_PINOCCHIO_SRC_SIZE_VISITOR_HH

namespace hpp {
  namespace pinocchio {
    namespace liegroupType {

      /// Visitor to compute size of LiegroupType
      struct SizeVisitor : public boost::static_visitor <>
      {
        inline SizeVisitor () : nq(-1), nv(-1) {}
        template <typename LiegroupType> void operator () (LiegroupType& op)
        {
          nq = op.nq ();
          nv = op.nv ();
        }
        size_type nq, nv;
      }; // struct SizeVisitor

      /// Visitor to compute neutral element of LiegroupType
      struct NeutralVisitor : public boost::static_visitor <>
      {
        template <typename LiegroupType> void operator () (LiegroupType& op)
        {
          neutral = op.neutral ();
        }
        vector_t neutral;
      }; // struct NeutralVisitor

      /// Visitor to check if a LiegroupType is a vector space
      struct IsVectorSpace : public boost::static_visitor <>
      {
        IsVectorSpace () : isVectorSpace (false) {}
        template <typename LiegroupType> void operator () (const LiegroupType&)
        {
          isVectorSpace = false;
        }
        template<int Size, bool rot>
        void operator () (const liegroup::VectorSpaceOperation<Size,rot>&)
        {
          isVectorSpace = true;
        }
        bool isVectorSpace;
      }; // struct SizeVisitor


    } // namespace liegroupType
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SRC_SIZE_VISITOR_HH
