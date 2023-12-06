// Copyright (c) 2017, Joseph Mirabel
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

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/parsers/sample-models.hpp>

namespace hpp {
namespace pinocchio {
template <typename Scalar, int Options,
          template <typename, int> class JointCollectionTpl>
void humanoidRandom(
    ::pinocchio::ModelTpl<Scalar, Options, JointCollectionTpl>& model) {
  using ::pinocchio::buildModels::details::addJointAndBody;
  static const SE3 Id = SE3::Identity();
  typedef JointCollectionTpl<Scalar, Options> JC;

  // root
  addJointAndBody(model, typename JC::JointModelFreeFlyer(), "universe", "root",
                  Id);
  model.lowerPositionLimit.template segment<4>(3).fill(-1.);
  model.upperPositionLimit.template segment<4>(3).fill(1.);

  // lleg
  addJointAndBody(model, typename JC::JointModelRX(), "root_joint", "lleg1");
  addJointAndBody(model, typename JC::JointModelRY(), "lleg1_joint", "lleg2");
  addJointAndBody(model, typename JC::JointModelRZ(), "lleg2_joint", "lleg3");
  addJointAndBody(model, typename JC::JointModelRY(), "lleg3_joint", "lleg4");
  addJointAndBody(model, typename JC::JointModelRY(), "lleg4_joint", "lleg5");
  addJointAndBody(model, typename JC::JointModelRX(), "lleg5_joint", "lleg6");

  // rleg
  addJointAndBody(model, typename JC::JointModelRX(), "root_joint", "rleg1");
  addJointAndBody(model, typename JC::JointModelRY(), "rleg1_joint", "rleg2");
  addJointAndBody(model, typename JC::JointModelRZ(), "rleg2_joint", "rleg3");
  addJointAndBody(model, typename JC::JointModelRY(), "rleg3_joint", "rleg4");
  addJointAndBody(model, typename JC::JointModelRY(), "rleg4_joint", "rleg5");
  addJointAndBody(model, typename JC::JointModelRX(), "rleg5_joint", "rleg6");

  // trunc
  addJointAndBody(model, typename JC::JointModelRY(), "root_joint", "torso1");
  addJointAndBody(model, typename JC::JointModelRZ(), "torso1_joint", "chest");

  // rarm
  addJointAndBody(model, typename JC::JointModelRX(), "chest_joint", "rarm1");
  addJointAndBody(model, typename JC::JointModelRY(), "rarm1_joint", "rarm2");
  addJointAndBody(model, typename JC::JointModelRZ(), "rarm2_joint", "rarm3");
  addJointAndBody(model, typename JC::JointModelRY(), "rarm3_joint", "rarm4");
  addJointAndBody(model, typename JC::JointModelRY(), "rarm4_joint", "rarm5");
  addJointAndBody(model, typename JC::JointModelRX(), "rarm5_joint", "rarm6");

  // larm
  addJointAndBody(model, typename JC::JointModelRX(), "chest_joint", "larm1");
  addJointAndBody(model, typename JC::JointModelRY(), "larm1_joint", "larm2");
  addJointAndBody(model, typename JC::JointModelRZ(), "larm2_joint", "larm3");
  addJointAndBody(model, typename JC::JointModelRY(), "larm3_joint", "larm4");
  addJointAndBody(model, typename JC::JointModelRY(), "larm4_joint", "larm5");
  addJointAndBody(model, typename JC::JointModelRX(), "larm5_joint", "larm6");
}

DevicePtr_t humanoidSimple(const std::string& name, bool usingFF,
                           Computation_t compFlags) {
  if (!usingFF)
    throw std::invalid_argument(
        "Humanoid simple without freefloating base is not supported anymore.");
  return humanoidSimple(name, compFlags);
}

DevicePtr_t humanoidSimple(const std::string& name, Computation_t compFlags) {
  DevicePtr_t robot = Device::create(name);
  humanoidRandom(robot->model());
  robot->createData();
  robot->createGeomData();
  robot->controlComputation(compFlags);
  robot->currentConfiguration(robot->neutralConfiguration());
  robot->computeForwardKinematics(compFlags);

  const value_type max = std::numeric_limits<value_type>::max();
  const value_type min = -std::numeric_limits<value_type>::max();
  robot->model().lowerPositionLimit.segment<3>(0).setConstant(min);
  robot->model().upperPositionLimit.segment<3>(0).setConstant(max);
  robot->model().lowerPositionLimit.segment<4>(3).setConstant(-1.01);
  robot->model().upperPositionLimit.segment<4>(3).setConstant(1.01);
  return robot;
}

namespace unittest {
DevicePtr_t makeDevice(TestDeviceType type, const std::string& prefix) {
  DevicePtr_t robot;
  HumanoidRobotPtr_t hrobot;
  (void)prefix;
  switch (type) {
    case CarLike:
      robot = Device::create("carlike");
      urdf::loadRobotModel(robot, 0, prefix, "planar", "hpp_environments",
                           "buggy", "", "");
      robot->model().lowerPositionLimit.head<2>().setConstant(-1);
      robot->model().upperPositionLimit.head<2>().setOnes();
      return robot;
    case ManipulatorArm2:
      robot = Device::create("arm");
      urdf::loadRobotModel(robot, 0, prefix, "anchor", "hpp_environments",
                           "tests/baxter", "_simple", "_simple");
      return robot;
    case HumanoidRomeo:
      hrobot = HumanoidRobot::create("romeo");
      urdf::loadRobotModel(hrobot, 0, prefix, "freeflyer", "romeo_description",
                           "romeo", "_small", "_small");
      urdf::setupHumanoidRobot(hrobot, prefix);
      hrobot->model().lowerPositionLimit.head<3>().setConstant(-1);
      hrobot->model().upperPositionLimit.head<3>().setOnes();
      return hrobot;
    case HumanoidSimple:
      robot = humanoidSimple("simple-humanoid",
                             (Computation_t)(JOINT_POSITION | JACOBIAN));
      robot->model().lowerPositionLimit.head<3>().setConstant(-1);
      robot->model().upperPositionLimit.head<3>().setOnes();
      return robot;
    default:
      throw std::invalid_argument("Unknown robot type.");
  }
}
}  // namespace unittest
}  // namespace pinocchio
}  // namespace hpp
