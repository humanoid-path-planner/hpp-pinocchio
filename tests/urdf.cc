// Copyright (c) 2019, Joseph Mirabel
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

#define BOOST_TEST_MODULE urdf

#include <boost/test/unit_test.hpp>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/urdf/util.hh>

using namespace hpp::pinocchio;

BOOST_AUTO_TEST_CASE (mimic_joint)
{
  std::string urdf (
      "<robot name='test'>"
      "<link name='base_link'/>"
      "<link name='link1'/>"
      "<joint name='joint1' type='revolute'>"
      "  <parent link='base_link'/>"
      "  <child link='link1'/>"
      "  <limit effort='30' velocity='1.0' />"
      "</joint>"
      "<link name='link2'/>"
      "<joint name='joint2' type='revolute'>"
      "  <parent link='base_link'/>"
      "  <child link='link2'/>"
      "  <mimic joint='joint1' multiplier='-1.' offset='0.5' />"
      "  <limit effort='30' velocity='1.0' />"
      "</joint>"
      "</robot>");
  DevicePtr_t robot = Device::create ("test");
  urdf::loadModelFromString (robot, 0, "test", "anchor", urdf, "");
  BOOST_REQUIRE(robot);

  BOOST_REQUIRE_EQUAL (robot->jointConstraints().size(), 1);
  BOOST_CHECK_EQUAL (robot->jointConstraints()[0].multiplier, -1.);
  BOOST_CHECK_EQUAL (robot->jointConstraints()[0].offset    , 0.5);
}
