// Copyright (c) 2022, CNRS
// Authors: Florent Lamiraux
//          Le Quang Anh
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

#ifndef HPP_PINOCCHIO_SRC_IS_NORMALIZED_VISITOR_HH
#define HPP_PINOCCHIO_SRC_IS_NORMALIZED_VISITOR_HH

namespace hpp {
namespace pinocchio {
namespace liegroupType {

/// Is Normalized visitor
template <typename vector_type>
struct IsNormalizedVisitor : public boost::static_visitor<> {
  IsNormalizedVisitor(const vector_type& e1, const value_type& eps, bool& res)
      : e1_(e1), iq_(0), eps_(eps), result(res) {}
  template <typename LiegroupType>
  void operator()(LiegroupType& op) {
    result &= op.isNormalized(
        e1_.template segment<LiegroupType::NQ>(iq_, op.nq()), eps_);

    iq_ += op.nq();
  }
  const vector_type& e1_;
  size_type iq_;
  const value_type& eps_;
  bool& result;
};  // struct IsNormalizedVisitor
}  // namespace liegroupType
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_SRC_IS_NORMALIZED_VISITOR_HH
