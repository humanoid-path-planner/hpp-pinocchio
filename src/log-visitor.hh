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

#ifndef HPP_PINOCCHIO_SRC_LOG_VISITOR_HH
#define HPP_PINOCCHIO_SRC_LOG_VISITOR_HH

#include <Eigen/Geometry>

namespace hpp {
namespace pinocchio {
namespace liegroupType {

typedef Eigen::Quaternion<value_type> quaternion_t;

/// Visitor to compute log of a LiegroupType instance
struct LogVisitor : public boost::static_visitor<> {
  LogVisitor(vectorIn_t v, vectorOut_t log) : v_(v), log(log) {}
  template <typename LiegroupType>
  void operator()(const LiegroupType& lg);
  vectorIn_t v_;
  vectorOut_t log;
};  // struct LogVisitor

template <>
inline void LogVisitor::operator()<liegroup::CartesianProductOperation<
    liegroup::VectorSpaceOperation<3, false>,
    liegroup::SpecialOrthogonalOperation<3> > >(
    const liegroup::CartesianProductOperation<
        liegroup::VectorSpaceOperation<3, false>,
        liegroup::SpecialOrthogonalOperation<3> >&) {
  log.head<3>() = v_.head<3>();
  quaternion_t q(v_.tail<4>());
  Eigen::AngleAxis<value_type> u(q);
  if (u.angle() > M_PI) {
    log.tail<3>() = -(2 * M_PI - u.angle()) * u.axis();
  } else {
    log.tail<3>() = u.axis() * u.angle();
  }
}

template <>
inline void LogVisitor::operator()<liegroup::CartesianProductOperation<
    liegroup::VectorSpaceOperation<2, false>,
    liegroup::SpecialOrthogonalOperation<2> > >(
    const liegroup::CartesianProductOperation<
        liegroup::VectorSpaceOperation<2, false>,
        liegroup::SpecialOrthogonalOperation<2> >&) {
  log.head<2>() = v_.head<2>();
  value_type c = v_[2], s = v_[3];
  log[2] = atan2(s, c);
}

template <>
inline void LogVisitor::operator()<liegroup::SpecialOrthogonalOperation<2> >(
    const liegroup::SpecialOrthogonalOperation<2>&) {
  value_type c = v_[0], s = v_[1];
  log[0] = atan2(s, c);
}

template <>
inline void LogVisitor::operator()<liegroup::SpecialOrthogonalOperation<3> >(
    const liegroup::SpecialOrthogonalOperation<3>&) {
  vector4_t tmp(v_);
  quaternion_t q(tmp);
  Eigen::AngleAxis<value_type> u(q);
  if (u.angle() > M_PI) {
    log = -(2 * M_PI - u.angle()) * u.axis();
  } else {
    log = u.axis() * u.angle();
  }
}

template <typename LiegroupType>
void LogVisitor::operator()(const LiegroupType&) {
  log = v_;
}
}  // namespace liegroupType
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_SRC_LOG_VISITOR_HH
