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

#ifndef HPP_PINOCCHIO_SRC_JDIFFERENCE_VISITOR_HH
#define HPP_PINOCCHIO_SRC_JDIFFERENCE_VISITOR_HH

#include <hpp/pinocchio/liegroup/vector-space.hh>

namespace hpp {
namespace pinocchio {
using ::pinocchio::ARG0;
using ::pinocchio::ARG1;
using ::pinocchio::ArgumentPosition;

namespace liegroupType {
template <ArgumentPosition arg, DerivativeProduct side>
struct dDifferenceVisitor : public boost::static_visitor<> {
  dDifferenceVisitor(vectorIn_t& q0, vectorIn_t& q1, matrixOut_t& J)
      : q0_(q0), q1_(q1), J_(J), iq_(0), iv_(0) {}

  template <typename LgT>
  void operator()(const LgT& lg) {
    typename LgT::JacobianMatrix_t Jint(lg.nv(), lg.nv());

    lg.template dDifference<arg>(q0_.segment<LgT::NQ>(iq_, lg.nq()),
                                 q1_.segment<LgT::NQ>(iq_, lg.nq()), Jint);
    if (side == DerivativeTimesInput)
      J_.middleRows<LgT::NV>(iv_, lg.nv()).applyOnTheLeft(Jint);
    else
      J_.middleCols<LgT::NV>(iv_, lg.nv()).applyOnTheRight(Jint);
    iq_ += lg.nq();
    iv_ += lg.nv();
  }

  template <int N, bool rot>
  void operator()(const liegroup::VectorSpaceOperation<N, rot>& lg) {
    if (arg == ARG0) {
      if (side == DerivativeTimesInput)
        J_.middleRows<N>(iv_, lg.nv()) *= -1;
      else
        J_.middleCols<N>(iv_, lg.nv()) *= -1;
    }
    iq_ += lg.nq();
    iv_ += lg.nv();
  }
  vectorIn_t &q0_, q1_;
  matrixOut_t& J_;
  size_type iq_, iv_;
};  // struct dDifferenceVisitor
}  // namespace liegroupType
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_SRC_JDIFFERENCE_VISITOR_HH
