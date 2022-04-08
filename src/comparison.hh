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

#ifndef HPP_PINOCCHIO_SRC_COMPARISON_HH
#define HPP_PINOCCHIO_SRC_COMPARISON_HH

#include <hpp/util/debug.hh>

namespace hpp {
namespace pinocchio {
namespace liegroup {

namespace level1 {
template <typename LgT1>
bool isEqual(const LgT1& lgt1, const LiegroupType& lgt2);

struct IsEqualVisitor : public boost::static_visitor<bool> {
  inline IsEqualVisitor(const LiegroupType& lg2);
  template <typename LgT1>
  inline bool operator()(const LgT1& lg1);

 private:
  const LiegroupType& lg2_;
};  // struct IsEqualVisitor
}  // namespace level1

namespace level2 {
template <typename LgT1>
struct IsEqualVisitor : public boost::static_visitor<bool> {
  inline IsEqualVisitor(const LgT1& lg1);
  template <typename LgT2>
  inline bool operator()(const LgT2& lg2);

 private:
  const LgT1& lg1_;
};  // struct IsEqualVisitor
}  // namespace level2

/// Default implementation: for pairs of different types
template <typename LgT1, typename LgT2>
struct Comparison {
  inline bool operator()(const LgT1&, const LgT2&);
};  // class Comparison

// Specialization for two instances of same type
template <typename LgT>
struct Comparison<LgT, LgT> {
  inline bool operator()(const LgT&, const LgT&);
};

// Specialization for vector spaces
template <bool rot>
struct Comparison<VectorSpaceOperation<Eigen::Dynamic, rot>,
                  VectorSpaceOperation<Eigen::Dynamic, rot> > {
  inline bool operator()(const VectorSpaceOperation<Eigen::Dynamic, rot>& lgt1,
                         const VectorSpaceOperation<Eigen::Dynamic, rot>& lgt2);
};

template <int Size, bool rot>
struct Comparison<VectorSpaceOperation<Eigen::Dynamic, rot>,
                  VectorSpaceOperation<Size, rot> > {
  inline bool operator()(const VectorSpaceOperation<Eigen::Dynamic, rot>& lgt1,
                         const VectorSpaceOperation<Size, rot>& lgt2);
};

template <int Size, bool rot>
struct Comparison<VectorSpaceOperation<Size, rot>,
                  VectorSpaceOperation<Eigen::Dynamic, rot> > {
  inline bool operator()(const VectorSpaceOperation<Size, rot>& lgt1,
                         const VectorSpaceOperation<Eigen::Dynamic, rot>& lgt2);
};

}  // namespace liegroup
}  // namespace pinocchio
}  // namespace hpp

#include "../src/comparison.hxx"

#endif
