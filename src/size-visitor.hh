// Copyright (c) 2017, CNRS
// Authors: Florent Lamiraux
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
