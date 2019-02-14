///
/// Copyright (c) 2019 CNRS
/// Author: Florent Lamiraux
///
///
// This file is part of hpp-pinocchio
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
// hpp-pinocchio  If not, see
// <http://www.gnu.org/licenses/>.

// This test
//   - sets bounds and checks that they are stored correctly

#define BOOST_TEST_MODULE joint-bounds

#include <boost/test/unit_test.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>

using hpp::pinocchio::size_type;
using hpp::pinocchio::JointPtr_t;
using hpp::pinocchio::vector_t;
using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::unittest::makeDevice;
using hpp::pinocchio::unittest::HumanoidSimple;

BOOST_AUTO_TEST_CASE(joint_bounds)
{
  DevicePtr_t robot;
  robot = makeDevice(HumanoidSimple);
  size_type nJoints (robot->nbJoints ());
  for (size_type i=0; i<nJoints; ++i) {
    JointPtr_t joint (robot->jointAt (i));
    // Set bounds
    vector_t l0 (joint->configSize ());
    vector_t u0 (joint->configSize ());
    l0.fill (-1.); u0.fill (1.);
    joint->lowerBounds (l0);
    joint->upperBounds (u0);
    // Get bounds
    vector_t l1 (joint->configSize ());
    vector_t u1 (joint->configSize ());
    for (size_type i=0; i<joint->configSize (); ++i) {
      l1 [i] = joint->lowerBound (i);
      u1 [i] = joint->upperBound (i);
    }
    BOOST_CHECK (l0 == l1);
    BOOST_CHECK (u0 == u1);
  }
}
