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

#ifndef HPP_PINOCCHIO_SRC_ADDITION_VISITOR_HH
#define HPP_PINOCCHIO_SRC_ADDITION_VISITOR_HH

#include <Eigen/Geometry>

namespace hpp {
namespace pinocchio {
namespace liegroupType {

typedef Eigen::Quaternion<value_type> quaternion_t;

/// Addition visitor
template <typename vector_type>
struct AdditionVisitor : public boost::static_visitor<> {
  AdditionVisitor(vector_type& e, const vectorIn_t& v)
      : e_(e), v_(v), iq_(0), iv_(0) {}
  template <typename LiegroupType>
  void operator()(LiegroupType& op) {
    op.integrate(e_.template segment<LiegroupType::NQ>(iq_, op.nq()),
                 v_.segment<LiegroupType::NV>(iv_, op.nv()),
                 e_.template segment<LiegroupType::NQ>(iq_, op.nq()));

    iq_ += op.nq();
    iv_ += op.nv();
  }
  vector_type& e_;
  const vectorIn_t& v_;
  size_type iq_, iv_;
};  // struct AdditionVisitor

}  // namespace liegroupType
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_SRC_ADDITION_VISITOR_HH
