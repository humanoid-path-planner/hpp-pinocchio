// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-pinocchio.
// hpp-pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-pinocchio. If not, see <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE tframe

#include <boost/test/unit_test.hpp>

#include <hpp/model/device.hh>

#include <pinocchio/algorithm/frames.hpp>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/frame.hh>
#include "../tests/utils.hh"

static bool verbose = false;

#define CHECK_TRANSFORM(M, Mexp) BOOST_CHECK((M.inverse() * Mexp).isIdentity())

using namespace hpp::pinocchio;

typedef std::vector<std::string> Strings_t;
void check_children(const Model& model, const Frame& f, Strings_t expected_children)
{
  const std::vector<FrameIndex>& children = f.children();
  BOOST_CHECK(children.size() == expected_children.size());
  for (std::size_t i = 0; i < children.size(); ++i) {
    std::string name = model.frames[children[i]].name;
    Strings_t::iterator _found = std::find(
          expected_children.begin(), expected_children.end(), name);
    BOOST_CHECK_MESSAGE(_found != expected_children.end(), "Child frame " << name << " not found");
    if (_found != expected_children.end())
      expected_children.erase(_found);
  }
  for (std::size_t i = 0; i < expected_children.size(); ++i) {
    BOOST_CHECK_MESSAGE(false, "Frame " << expected_children[i] << " not found");
  }
}

BOOST_AUTO_TEST_CASE (frame)
{
  DevicePtr_t pinocchio = hppPinocchio();
  Configuration_t q = pinocchio->neutralConfiguration();
  pinocchio->currentConfiguration(q);
  pinocchio->computeForwardKinematics();

  Frame root  = pinocchio->getFrameByName("root_joint");
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
  check_children(pinocchio->model(), root , Strings_t());

  // Check position in parent frame.
  Joint root_j = waist.joint();
  BOOST_CHECK(root_j.name() == "root_joint");

  Frame
    ImuTorsoAcc_F =  pinocchio->getFrameByName("ImuTorsoAccelerometer_joint"),
    ImuTorsoGyr_F =  pinocchio->getFrameByName("ImuTorsoGyrometer_joint"),
    TrunkYaw_F    =  pinocchio->getFrameByName("TrunkYaw"),
    LHipYaw_F     =  pinocchio->getFrameByName("LHipYaw"),
    RHipYaw_F     =  pinocchio->getFrameByName("RHipYaw");

  Transform3f
    ImuTorsoAcc_M =  ImuTorsoAcc_F.positionInParentFrame(),
    ImuTorsoGyr_M =  ImuTorsoGyr_F.positionInParentFrame(),
    TrunkYaw_M    =  TrunkYaw_F   .positionInParentFrame(),
    LHipYaw_M     =  LHipYaw_F    .positionInParentFrame(),
    RHipYaw_M     =  RHipYaw_F    .positionInParentFrame(),
    waist_M       =  waist        .positionInParentFrame();

  Transform3f shift = Transform3f::Random();
  waist.positionInParentFrame(shift);
  pinocchio->computeForwardKinematics();

  CHECK_TRANSFORM(ImuTorsoAcc_F.positionInParentFrame(), ImuTorsoAcc_M);
  CHECK_TRANSFORM(ImuTorsoGyr_F.positionInParentFrame(), ImuTorsoGyr_M);
  CHECK_TRANSFORM(TrunkYaw_F   .positionInParentFrame(), TrunkYaw_M   );
  CHECK_TRANSFORM(LHipYaw_F    .positionInParentFrame(), LHipYaw_M    );
  CHECK_TRANSFORM(RHipYaw_F    .positionInParentFrame(), RHipYaw_M    );
  CHECK_TRANSFORM(waist        .positionInParentFrame(), shift        );

  CHECK_TRANSFORM(ImuTorsoAcc_F.currentTransformation(), shift * ImuTorsoAcc_M);
  CHECK_TRANSFORM(ImuTorsoGyr_F.currentTransformation(), shift * ImuTorsoGyr_M);
  CHECK_TRANSFORM(TrunkYaw_F   .currentTransformation(), shift * TrunkYaw_M   );
  CHECK_TRANSFORM(LHipYaw_F    .currentTransformation(), shift * LHipYaw_M    );
  CHECK_TRANSFORM(RHipYaw_F    .currentTransformation(), shift * RHipYaw_M    );
}
