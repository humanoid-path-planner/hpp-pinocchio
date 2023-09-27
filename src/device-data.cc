// Copyright (c) 2018, Joseph Mirabel
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

#include <hpp/pinocchio/device-data.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

namespace hpp {
namespace pinocchio {

DeviceData::DeviceData() { invalidate(); }

DeviceData::DeviceData(const DeviceData& other)
    : data_(new Data(*other.data_)),
      geomData_(new GeomData(*other.geomData_)),
      currentConfiguration_(other.currentConfiguration_),
      currentVelocity_(other.currentVelocity_),
      currentAcceleration_(other.currentAcceleration_),
      dataUpToDate_(other.dataUpToDate_),
      frameUpToDate_(other.frameUpToDate_),
      geomUpToDate_(other.geomUpToDate_),
      devicePtr_(),
      modelConf_(other.modelConf_.size()),
      jointJacobians_(other.jointJacobians_.size()) {}

void checkComputationFlag(int flag) {
  // Turn off unused parameter compilation warning.
  (void)flag;
  // a IMPLIES b === (b || ~a)
  // velocity IMPLIES position
  assert((flag & JOINT_POSITION) || (!(flag & VELOCITY)));
  // acceleration IMPLIES velocity
  assert((flag & VELOCITY) || (!(flag & ACCELERATION)));
  // com IMPLIES position
  assert((flag & JOINT_POSITION) || (!(flag & COM)));
  // jacobian IMPLIES position
  assert((flag & JOINT_POSITION) || (!(flag & JACOBIAN)));
}

constexpr int FK_ACC = JOINT_POSITION | VELOCITY | ACCELERATION;
constexpr int FK_VEL = JOINT_POSITION | VELOCITY;
constexpr int FK_POS = JOINT_POSITION;
// JCOM is different from JACOBIAN | COM
// as one could do
// computeForwardKinematics(JACOBIAN);
// computeForwardKinematics(COM);
// computeForwardKinematics(COM | JACOBIAN);
// In the last call, if JCOM = COM | JACOBIAN, the COM Jacobian would not be
// computed.
constexpr int JCOM = COM << 1 | COM;

void DeviceData::computeForwardKinematics(const ModelPtr_t& mptr, int flag) {
  checkComputationFlag(flag);

  const Model& model = *mptr;
  const size_type nq = model.nq;
  const size_type nv = model.nv;

  // TODO pinocchio does not allow to pass currentConfiguration_.head(nq) as
  // a reference. This line avoids dynamic memory allocation
  modelConf_ = currentConfiguration_.head(nq);

  switch (flag & FK_ACC) {
    case FK_ACC:
      if (!(dataUpToDate_ & ACCELERATION)) {
        ::pinocchio::forwardKinematics(model, *data_, modelConf_,
                                       currentVelocity_.head(nv),
                                       currentAcceleration_.head(nv));
        dataUpToDate_ = dataUpToDate_ | FK_ACC;
      }
      break;
    case FK_VEL:
      if (!(dataUpToDate_ & VELOCITY)) {
        ::pinocchio::forwardKinematics(model, *data_, modelConf_,
                                       currentVelocity_.head(nv));
        dataUpToDate_ = dataUpToDate_ | FK_VEL;
      }
      break;
    case FK_POS:
      if (!(dataUpToDate_ & JOINT_POSITION)) {
        ::pinocchio::forwardKinematics(model, *data_, modelConf_);
        dataUpToDate_ = dataUpToDate_ | FK_POS;
      }
      break;
    default:
      throw std::logic_error("Wrong computation flag.");
  }

  if (flag & COM) {
    if (flag & JACOBIAN) {
      if (!(dataUpToDate_ & JCOM)) {
        ::pinocchio::jacobianCenterOfMass(model, *data_, modelConf_, true);
        dataUpToDate_ = dataUpToDate_ | JCOM | COM;
      }
    } else {
      if (!(dataUpToDate_ & COM)) {
        ::pinocchio::centerOfMass(model, *data_, ::pinocchio::POSITION, true);
        dataUpToDate_ = dataUpToDate_ | COM;
      }
    }
  }

  if (flag & JACOBIAN) {
    if (!(dataUpToDate_ & JACOBIAN)) {
      ::pinocchio::computeJointJacobians(model, *data_, modelConf_);
      dataUpToDate_ = dataUpToDate_ | JACOBIAN;
    }
  }
}

void DeviceData::computeFramesForwardKinematics(const ModelPtr_t& mptr) {
  if (frameUpToDate_) return;
  computeForwardKinematics(mptr, JOINT_POSITION);

  ::pinocchio::updateFramePlacements(*mptr, *data_);

  frameUpToDate_ = true;
}

void DeviceData::updateGeometryPlacements(const ModelPtr_t& m,
                                          const GeomModelPtr_t& gm) {
  if (!geomUpToDate_) {
    ::pinocchio::updateGeometryPlacements(*m, *data_, *gm, *geomData_);
    geomUpToDate_ = true;
  }
}
}  // namespace pinocchio
}  // namespace hpp
