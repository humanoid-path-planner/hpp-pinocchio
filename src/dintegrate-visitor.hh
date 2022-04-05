// Copyright (c) 2018, CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#ifndef HPP_PINOCCHIO_SRC_DINTEGRATE_VISITOR_HH
#define HPP_PINOCCHIO_SRC_DINTEGRATE_VISITOR_HH

#include <hpp/pinocchio/liegroup/vector-space.hh>

namespace hpp {
namespace pinocchio {
namespace liegroupType {
template <int arg, DerivativeProduct side>
struct dIntegrateVisitor : public boost::static_visitor<> {
  dIntegrateVisitor(const vectorIn_t& q, vectorIn_t& v, matrixOut_t& Jq,
                    size_type& row, size_type& configRow)
      : q_(q), v_(v), Jq_(Jq), row_(row), configRow_(configRow) {}

  template <typename LgT>
  void operator()(const LgT& lg) {
    // TODO add static asserts
    assert(arg == 0 || arg == 1);
    typename LgT::JacobianMatrix_t JInt(lg.nv(), lg.nv());
    if (arg == 0)
      lg.dIntegrate_dq(q_.segment<LgT::NQ>(configRow_, lg.nq()),
                       v_.segment<LgT::NV>(row_, lg.nv()), JInt);
    else
      lg.dIntegrate_dv(q_.segment<LgT::NQ>(configRow_, lg.nq()),
                       v_.segment<LgT::NV>(row_, lg.nv()), JInt);
    if (side == DerivativeTimesInput)
      Jq_.middleRows<LgT::NV>(row_, lg.nv()).applyOnTheLeft(JInt);
    else
      Jq_.middleCols<LgT::NV>(row_, lg.nv()).applyOnTheRight(JInt);
    row_ += lg.nv();
    configRow_ += lg.nq();
  }

  template <int N, bool rot>
  void operator()(const liegroup::VectorSpaceOperation<N, rot>& lg) {
    row_ += lg.nv();
    configRow_ += lg.nq();
  }

  const vectorIn_t& q_;
  vectorIn_t& v_;
  matrixOut_t& Jq_;
  size_type& row_;
  size_type& configRow_;
};  // struct dIntegrateVisitor
}  // namespace liegroupType
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_SRC_JINTEGRATE_VISITOR_HH
