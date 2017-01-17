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

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/frame.hh>
#include "../tests/utils.hh"

static bool verbose = false;

using namespace hpp::pinocchio;

BOOST_AUTO_TEST_CASE (frame)
{
  DevicePtr_t pinocchio = hppPinocchio();

  Frame waist = pinocchio->getFrameByName("waist");
  BOOST_CHECK(waist.isFixed());
  BOOST_CHECK(waist.parentFrame().name() == "root_joint");
  BOOST_CHECK(!waist.parentFrame().isFixed());

  std::vector<std::string> expected_children;
  expected_children.push_back("ImuTorsoAccelerometer_joint");
  expected_children.push_back("ImuTorsoGyrometer_joint");
  expected_children.push_back("TrunkYaw");
  expected_children.push_back("LHipYaw");
  expected_children.push_back("RHipYaw");

  const std::vector<FrameIndex>& children = waist.children();
  BOOST_CHECK(children.size() == expected_children.size());
  for (std::size_t i = 0; i < children.size(); ++i) {
    std::string name = pinocchio->model().frames[children[i]].name;
      std::vector<std::string>::iterator _found = std::find(
          expected_children.begin(), expected_children.end(), name);
    BOOST_CHECK_MESSAGE(_found != expected_children.end(), "Child frame " << name << " not found");
    if (_found != expected_children.end())
      expected_children.erase(_found);
  }
  for (std::size_t i = 0; i < expected_children.size(); ++i) {
    BOOST_CHECK_MESSAGE(false, "Frame " << expected_children[i] << " not found");
  }
}
