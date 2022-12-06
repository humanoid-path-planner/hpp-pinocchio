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

#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>

namespace hpp {
namespace pinocchio {

CollisionObject::CollisionObject(DevicePtr_t device,
                                 const GeomIndex geomInModel)
    : devicePtr(device),
      geomModel_(devicePtr->geomModelPtr()),
      jointIndex_(0),
      geomInModelIndex(geomInModel) {
  jointIndex_ = pinocchio().parentJoint;
  selfAssert();
}

CollisionObject::CollisionObject(GeomModelPtr_t geomModel,
                                 GeomDataPtr_t geomData,
                                 const GeomIndex geomInModel)
    : devicePtr(),
      geomModel_(geomModel),
      geomData_(geomData),
      jointIndex_(0),
      geomInModelIndex(geomInModel) {
  jointIndex_ = pinocchio().parentJoint;
  selfAssert();
}

const std::string& CollisionObject::name() const { return pinocchio().name; }

// This function should rather return a shared_ptr<const>
const ::pinocchio::GeometryObject& CollisionObject::pinocchio() const {
  return geomModel_->geometryObjects[geomInModelIndex];
}
::pinocchio::GeometryObject& CollisionObject::pinocchio() {
  return geomModel_->geometryObjects[geomInModelIndex];
}

CollisionGeometryPtr_t CollisionObject::geometry() const {
  return pinocchio().geometry;
}

JointPtr_t CollisionObject::joint() {
  if (!devicePtr) return JointPtr_t();
  return Joint::create(devicePtr, jointIndex_);
}

JointConstPtr_t CollisionObject::joint() const {
  if (!devicePtr) return JointConstPtr_t();
  return Joint::create(devicePtr, jointIndex_);
}

const Transform3f& CollisionObject::positionInJointFrame() const {
  return pinocchio().placement;
}

fcl::Transform3f CollisionObject::getFclTransform() const {
  return ::pinocchio::toFclTransform3f(geomData().oMg[geomInModelIndex]);
}
const Transform3f& CollisionObject::getTransform() const {
  return geomData().oMg[geomInModelIndex];
}
const Transform3f& CollisionObject::getTransform(const DeviceData& d) const {
  return geomData(d).oMg[geomInModelIndex];
}

void CollisionObject::move(const Transform3f& position) {
  // move does not work but for object attached to the universe (joint 0)
  assert(jointIndex_ == 0);
  geomData().oMg[geomInModelIndex] = position;
  pinocchio().placement = position;
}

void CollisionObject::selfAssert() const {
  assert(geomModel_);
  assert(devicePtr || geomData_);
  assert(!devicePtr ||
         devicePtr->model().joints.size() > std::size_t(jointIndex_));
  assert(geomModel_->geometryObjects.size() > geomInModelIndex);
}

GeomData& CollisionObject::geomData() const {
  if (devicePtr)  // Object of the robot.
    return geomData(devicePtr->d());
  else  // Object of the environment.
    return *geomData_;
}

GeomData& CollisionObject::geomData(DeviceData& d) const {
  if (geomData_)
    return *geomData_;
  else
    return *d.geomData_;
}

const GeomData& CollisionObject::geomData(const DeviceData& d) const {
  if (geomData_)
    return *geomData_;
  else
    return *d.geomData_;
}
}  // namespace pinocchio
}  // namespace hpp
