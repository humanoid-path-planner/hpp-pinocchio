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

#define BOOST_TEST_MODULE tframe

#include <boost/test/unit_test.hpp>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/frame.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/algorithm/frames.hpp>

#define CHECK_TRANSFORM(M, Mexp) BOOST_CHECK((M.inverse() * Mexp).isIdentity())

using namespace hpp::pinocchio;

typedef std::vector<std::string> Strings_t;
void check_children(const Model& model, const Frame& f,
                    Strings_t expected_children) {
  const std::vector<FrameIndex>& children = f.children();
  BOOST_CHECK(children.size() == expected_children.size());
  for (std::size_t i = 0; i < children.size(); ++i) {
    std::string name = model.frames[children[i]].name;
    Strings_t::iterator _found =
        std::find(expected_children.begin(), expected_children.end(), name);
    BOOST_CHECK_MESSAGE(_found != expected_children.end(),
                        "Child frame " << name << " not found");
    if (_found != expected_children.end()) expected_children.erase(_found);
  }
  for (std::size_t i = 0; i < expected_children.size(); ++i) {
    BOOST_CHECK_MESSAGE(false,
                        "Frame " << expected_children[i] << " not found");
  }
}

/* Build a hpp::pinocchio::Device from urdf path. */
DevicePtr_t hppPinocchio() {
  return unittest::makeDevice(unittest::HumanoidRomeo);
}

BOOST_AUTO_TEST_CASE(frame) {
  DevicePtr_t pinocchio = hppPinocchio();
  Configuration_t q = pinocchio->neutralConfiguration();
  pinocchio->currentConfiguration(q);
  pinocchio->computeForwardKinematics();

  Frame root = pinocchio->getFrameByName("root_joint");
  Frame waist = pinocchio->getFrameByName("waist");
  BOOST_CHECK(!root.isFixed());
  BOOST_CHECK(waist.isFixed());
  BOOST_CHECK(waist.parentFrame().name() == "root_joint");
  BOOST_CHECK(!waist.parentFrame().isFixed());

  Strings_t expected_children;
  expected_children.push_back("ImuTorsoAccelerometer_frame");
  expected_children.push_back("ImuTorsoGyrometer_frame");
  expected_children.push_back("TrunkYaw");
  expected_children.push_back("LHipYaw");
  expected_children.push_back("RHipYaw");
  expected_children.push_back("body");

  // Check that the children are properly set.
  check_children(pinocchio->model(), waist, expected_children);
  check_children(pinocchio->model(), root, Strings_t());

  // Check position in parent frame.
  JointPtr_t root_j = waist.joint();
  BOOST_REQUIRE(root_j);
  BOOST_CHECK(root_j->name() == "root_joint");

  Frame ImuTorsoAcc_F =
            pinocchio->getFrameByName("ImuTorsoAccelerometer_joint"),
        ImuTorsoGyr_F = pinocchio->getFrameByName("ImuTorsoGyrometer_joint"),
        TrunkYaw_F = pinocchio->getFrameByName("TrunkYaw"),
        LHipYaw_F = pinocchio->getFrameByName("LHipYaw"),
        RHipYaw_F = pinocchio->getFrameByName("RHipYaw");

  Transform3f ImuTorsoAcc_M = ImuTorsoAcc_F.positionInParentFrame(),
              ImuTorsoGyr_M = ImuTorsoGyr_F.positionInParentFrame(),
              TrunkYaw_M = TrunkYaw_F.positionInParentFrame(),
              LHipYaw_M = LHipYaw_F.positionInParentFrame(),
              RHipYaw_M = RHipYaw_F.positionInParentFrame();

  Transform3f shift = Transform3f::Random();
  waist.positionInParentFrame(shift);
  pinocchio->computeForwardKinematics();

  CHECK_TRANSFORM(ImuTorsoAcc_F.positionInParentFrame(), ImuTorsoAcc_M);
  CHECK_TRANSFORM(ImuTorsoGyr_F.positionInParentFrame(), ImuTorsoGyr_M);
  CHECK_TRANSFORM(TrunkYaw_F.positionInParentFrame(), TrunkYaw_M);
  CHECK_TRANSFORM(LHipYaw_F.positionInParentFrame(), LHipYaw_M);
  CHECK_TRANSFORM(RHipYaw_F.positionInParentFrame(), RHipYaw_M);
  CHECK_TRANSFORM(waist.positionInParentFrame(), shift);

  CHECK_TRANSFORM(ImuTorsoAcc_F.currentTransformation(), shift * ImuTorsoAcc_M);
  CHECK_TRANSFORM(ImuTorsoGyr_F.currentTransformation(), shift * ImuTorsoGyr_M);
  CHECK_TRANSFORM(TrunkYaw_F.currentTransformation(), shift * TrunkYaw_M);
  CHECK_TRANSFORM(LHipYaw_F.currentTransformation(), shift * LHipYaw_M);
  CHECK_TRANSFORM(RHipYaw_F.currentTransformation(), shift * RHipYaw_M);
}
