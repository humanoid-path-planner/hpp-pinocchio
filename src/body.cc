//
// Copyright (c) 2016 CNRS
// Author: NMansard
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

#include <boost/foreach.hpp>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>

namespace hpp {
namespace pinocchio {

Body::Body(DeviceWkPtr_t device, JointIndex joint)
    : devicePtr(device), jointIndex(joint), frameIndexSet(false) {
  selfAssert();
}

void Body::selfAssert() const {
  DevicePtr_t device = devicePtr.lock();
  assert(device);
  assert(device->modelPtr());
  assert(device->model().joints.size() > std::size_t(jointIndex));
  if (frameIndexSet)
    assert(device->model().frames.size() > std::size_t(frameIndex));
}

const Model& Body::model() const { return devicePtr.lock()->model(); }
Model& Body::model() { return devicePtr.lock()->model(); }
::pinocchio::Frame& Body::frame() {
  searchFrameIndex();
  return model().frames[frameIndex];
}
const ::pinocchio::Frame& Body::frame() const {
  searchFrameIndex();
  return model().frames[frameIndex];
}

void Body::searchFrameIndex() const {
  if (frameIndexSet) return;
  frameIndex = 0;
  BOOST_FOREACH (const ::pinocchio::Frame& frame, model().frames) {
    if ((::pinocchio::BODY == frame.type) && (frame.parent == jointIndex))
      break;
    frameIndex++;
  }
  // If index not find, then it is set to size() -> normal behavior
  frameIndexSet = true;
}

const std::string& Body::name() const {
  selfAssert();
  return frame().name;
}

JointPtr_t Body::joint() const {
  selfAssert();
  return Joint::create(devicePtr, jointIndex);
}

const vector3_t& Body::localCenterOfMass() const {
  selfAssert();
  return model().inertias[jointIndex].lever();
}
matrix3_t Body::inertiaMatrix() const {
  selfAssert();
  return model().inertias[jointIndex].inertia();
}
value_type Body::mass() const {
  selfAssert();
  return model().inertias[jointIndex].mass();
}

size_type Body::nbInnerObjects() const {
  selfAssert();
  DevicePtr_t device = devicePtr.lock();
  return (size_type)device->geomData().innerObjects[jointIndex].size();
}

CollisionObjectPtr_t Body::innerObjectAt(const size_type& i) const {
  selfAssert();
  assert(0 <= i && i < nbInnerObjects());
  DevicePtr_t device = devicePtr.lock();
  return CollisionObjectPtr_t(new CollisionObject(
      device, device->geomData().innerObjects[jointIndex][(std::size_t)i]));
}

value_type Body::radius() const {
  selfAssert();
  DevicePtr_t device = devicePtr.lock();
  assert(device->geomDataPtr());
  assert(std::size_t(device->geomData().radius.size()) ==
         model().joints.size());
  return device->geomData().radius[jointIndex];
}

size_type Body::nbOuterObjects() const {
  selfAssert();
  DevicePtr_t device = devicePtr.lock();
  return (size_type)device->geomData().outerObjects[jointIndex].size();
}

CollisionObjectPtr_t Body::outerObjectAt(const size_type& i) const {
  selfAssert();
  assert(0 <= i && i < nbOuterObjects());
  DevicePtr_t device = devicePtr.lock();
  return CollisionObjectPtr_t(new CollisionObject(
      device, device->geomData().outerObjects[jointIndex][(std::size_t)i]));
}
}  // namespace pinocchio
}  // namespace hpp
