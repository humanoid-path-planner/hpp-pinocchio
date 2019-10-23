// Copyright (c) 2019, Joseph Mirabel
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
