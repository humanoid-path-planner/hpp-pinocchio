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
