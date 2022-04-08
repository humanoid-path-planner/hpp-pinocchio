///
/// Copyright (c) 2019 CNRS
/// Author: Florent Lamiraux
///
///

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

// This test
//   - sets bounds and checks that they are stored correctly

#define BOOST_TEST_MODULE joint - bounds

#include <boost/test/unit_test.hpp>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>

using hpp::pinocchio::DevicePtr_t;
using hpp::pinocchio::JointPtr_t;
using hpp::pinocchio::size_type;
using hpp::pinocchio::vector_t;
using hpp::pinocchio::unittest::HumanoidSimple;
using hpp::pinocchio::unittest::makeDevice;

BOOST_AUTO_TEST_CASE(joint_bounds) {
  DevicePtr_t robot;
  robot = makeDevice(HumanoidSimple);
  size_type nJoints(robot->nbJoints());
  for (size_type i = 0; i < nJoints; ++i) {
    JointPtr_t joint(robot->jointAt(i));
    BOOST_CHECK(joint == joint);
    BOOST_CHECK(*joint == *robot->jointAt(i));
    if (i > 0) BOOST_CHECK(*joint != *robot->jointAt(i - 1));

    // Set bounds
    vector_t l0(joint->configSize());
    vector_t u0(joint->configSize());
    l0.fill(-1.);
    u0.fill(1.);
    joint->lowerBounds(l0);
    joint->upperBounds(u0);
    // Get bounds
    vector_t l1(joint->configSize());
    vector_t u1(joint->configSize());
    for (size_type i = 0; i < joint->configSize(); ++i) {
      l1[i] = joint->lowerBound(i);
      u1[i] = joint->upperBound(i);
    }
    BOOST_CHECK(l0 == l1);
    BOOST_CHECK(u0 == u1);
  }
}
