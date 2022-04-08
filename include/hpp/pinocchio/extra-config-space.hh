//
// Copyright (c) 2013, 2014 CNRS
// Author: Florent Lamiraux
//
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

#ifndef HPP_PINOCCHIO_EXTRA_CONFIG_SPACE_HH
#define HPP_PINOCCHIO_EXTRA_CONFIG_SPACE_HH

#include <hpp/pinocchio/fwd.hh>

namespace hpp {
namespace pinocchio {
/// Extra degrees of freedom to store internal values in configurations
///
/// In some applications, it is useful to store extra variables
/// with the configuration vector of a robot. For instance, when
/// planning motions in state space using roadmap based methods,
/// the velocity of the robot is stored in the nodes of the
/// roadmap.
class ExtraConfigSpace {
 public:
  ExtraConfigSpace() : dimension_(0), lowerBounds_(), upperBounds_() {
    lowerBounds_.resize(0);
    upperBounds_.resize(0);
  }
  value_type& lower(const size_type& index) { return lowerBounds_[index]; }
  value_type& upper(const size_type& index) { return upperBounds_[index]; }
  const value_type& lower(const size_type& index) const {
    return lowerBounds_[index];
  }
  const value_type& upper(const size_type& index) const {
    return upperBounds_[index];
  }
  const vector_t& lower() const { return lowerBounds_; }
  const vector_t& upper() const { return upperBounds_; }
  /// Get dimension
  size_type dimension() const { return dimension_; }

 private:
  /// Set dimension of extra configuration space
  ///
  /// resize lowerBounds and upperBounds, set bounds to -infinity, +infinity
  void setDimension(const size_type& dimension) {
    dimension_ = dimension;
    lowerBounds_.resize(dimension);
    upperBounds_.resize(dimension);
    lowerBounds_.setConstant(-std::numeric_limits<value_type>::infinity());
    upperBounds_.setConstant(+std::numeric_limits<value_type>::infinity());
  }
  size_type dimension_;
  vector_t lowerBounds_;
  vector_t upperBounds_;
  friend class Device;
};  // class ExtraConfigSpace
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_EXTRA_CONFIG_SPACE_HH
