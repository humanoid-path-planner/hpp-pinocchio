//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel from Florent Lamiraux
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

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint.hh>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace hpp {
namespace pinocchio {
Gripper::Gripper(const std::string& name, const DeviceWkPtr_t& device)
    : name_(name), device_(device), clearance_(0) {
  DevicePtr_t d = this->device();
  fid_ = d->model().getFrameId(name);
  hppDout(info, "Creating gripper " << name << " with frame id " << fid_);
  joint_ = Joint::create(d, d->model().frames[fid_].parent);
}

const Transform3f& Gripper::objectPositionInJoint() const {
  // Check that the rank of the gripper frame has not been modified after appending other
  // models
  const Model& model(this->device()->model());
  if (model.frames[fid_].name != name_) fid_ = model.getFrameId(name_);
  return model.frames[fid_].placement;
}

GripperPtr_t Gripper::clone() const { return Gripper::create(name_, device_); }

std::ostream& Gripper::print(std::ostream& os) const {
  os << "name :" << name() << std::endl;
  os << "Position in joint :" << objectPositionInJoint();
  os << "Joint :" << (joint() ? joint()->name() : "universe") << std::endl;
  os << std::endl;
  return os;
}

DevicePtr_t Gripper::device() const {
  DevicePtr_t d = device_.lock();
  assert(d);
  return d;
}
}  // namespace pinocchio
}  // namespace hpp
