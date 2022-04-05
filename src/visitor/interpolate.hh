// Copyright (c) 2020, CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_SRC_VISITOR_INTERPOLATE_HH
#define HPP_PINOCCHIO_SRC_VISITOR_INTERPOLATE_HH

namespace hpp {
namespace pinocchio {
namespace liegroupType {
namespace visitor {
/// Interpolate
struct Interpolate : public boost::static_visitor<> {
  Interpolate(const vectorIn_t& e1, const vectorIn_t& e2, value_type u,
              vectorOut_t& er)
      : e1_(e1), e2_(e2), u_(u), er_(er), iq_(0), iv_(0) {}
  template <typename LiegroupType>
  void operator()(LiegroupType& op) {
    op.interpolate(e1_.segment<LiegroupType::NQ>(iq_, op.nq()),
                   e2_.segment<LiegroupType::NQ>(iq_, op.nq()), u_,
                   er_.segment<LiegroupType::NQ>(iq_, op.nq()));

    iq_ += op.nq();
    iv_ += op.nv();
  }
  const vectorIn_t& e1_;
  const vectorIn_t& e2_;
  value_type u_;
  vectorOut_t& er_;
  size_type iq_, iv_;
};  // struct SubstractionVisitor
}  // namespace visitor
}  // namespace liegroupType
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_SRC_VISITOR_INTERPOLATE_HH
