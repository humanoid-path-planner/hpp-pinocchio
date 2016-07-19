// Copyright (c) 2016, Joseph Mirabel
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

#define BOOST_TEST_MODULE tgripper

#include <boost/test/unit_test.hpp>

#include <hpp/model/gripper.hh>
#include <hpp/model/device.hh>
#include <hpp/model/urdf/util.hh>

#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/device.hh>

#include "../tests/utils.hh"

static bool verbose = false;

BOOST_AUTO_TEST_CASE(gripper)
{
  hpp::model::DevicePtr_t     model     = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  std::string jointName   = "LWristPitch";
  std::string gripperName = "gripper_name";
  hpp::model    ::Transform3f mT; mT.setIdentity();
  hpp::pinocchio::Transform3f pT; pT.setIdentity();

  /// Create gripper with model
  hpp::model::GripperPtr_t mGripper =
    hpp::model::Gripper::create (gripperName,
        model->getJointByName (jointName), mT,
        hpp::model::JointVector_t());

  /// Create gripper with Pinocchio
  pinocchio->model()->addFrame(gripperName,
      pinocchio->model()->getJointId(jointName),
      pT);
  hpp::pinocchio::GripperPtr_t pGripper =
    hpp::pinocchio::Gripper::create (gripperName, pinocchio);

  BOOST_CHECK(  m2p::SE3(mGripper->objectPositionInJoint())
      .isApprox(         pGripper->objectPositionInJoint()) );


  gripperName += "_2";
  pT = hpp::pinocchio::Transform3f::Random();
  mT = p2m::SE3(pT);

  /// Create gripper with model
  mGripper = hpp::model::Gripper::create (gripperName,
        model->getJointByName (jointName), mT,
        hpp::model::JointVector_t());

  /// Create gripper with Pinocchio
  pinocchio->model()->addFrame(gripperName,
      pinocchio->model()->getJointId(jointName),
      pT);
  pGripper = hpp::pinocchio::Gripper::create (gripperName, pinocchio);

  BOOST_CHECK(  m2p::SE3(mGripper->objectPositionInJoint())
      .isApprox(         pGripper->objectPositionInJoint()) );
}
