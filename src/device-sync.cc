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

#include <boost/thread/locks.hpp>
#include <hpp/pinocchio/device-sync.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

namespace hpp {
namespace pinocchio {
bool AbstractDevice::currentConfiguration(ConfigurationIn_t configuration) {
  DeviceData& data = d();
  assert(configuration.size() == data.currentConfiguration_.size());
  if (configuration != data.currentConfiguration_) {
    data.invalidate();
    data.currentConfiguration_ = configuration;
    return true;
  }
  return false;
}

bool AbstractDevice::currentVelocity(vectorIn_t v) {
  DeviceData& data = d();
  if (v != data.currentVelocity_) {
    data.invalidate();
    data.currentVelocity_ = v;
    return true;
  }
  return false;
}

bool AbstractDevice::currentAcceleration(vectorIn_t a) {
  DeviceData& data = d();
  if (a != data.currentAcceleration_) {
    data.invalidate();
    data.currentAcceleration_ = a;
    return true;
  }
  return false;
}

const value_type& AbstractDevice::mass() const { return data().mass[0]; }

const vector3_t& AbstractDevice::positionCenterOfMass() const {
  return data().com[0];
}

const ComJacobian_t& AbstractDevice::jacobianCenterOfMass() const {
  return data().Jcom;
}

void AbstractDevice::controlComputation(const Computation_t& flag) {
  if (d().computationFlag_ != flag) {
    d().computationFlag_ = flag;
    d().invalidate();
  }
}

AbstractDevice::AbstractDevice()
    : model_(new Model()), geomModel_(new GeomModel()) {}

AbstractDevice::AbstractDevice(const ModelPtr_t& m, const GeomModelPtr_t& gm)
    : model_(m), geomModel_(gm) {}

DeviceSync::DeviceSync(const DevicePtr_t& d, bool acquireLock)
    : AbstractDevice(d->modelPtr(), d->geomModelPtr()), device_(d), d_(NULL) {
  if (acquireLock) lock();
}

DeviceSync::~DeviceSync() {
  if (isLocked()) unlock();
}

void DeviceSync::lock() {
  if (!isLocked()) {
    d_ = device_->datas_.acquire();
  } else {
    hppDout(warning,
            "Cannot lock a locked DeviceSync. You may a concurrency error.");
  }
}

void DeviceSync::unlock() {
  if (isLocked()) {
    device_->datas_.release(d_);
    d_ = NULL;
  } else {
    hppDout(
        warning,
        "Cannot unlock an unlocked DeviceSync. You may a concurrency error.");
  }
}
}  // namespace pinocchio
}  // namespace hpp
