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

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/pinocchio/liegroup-space.hh>

static bool verbose = true;

using namespace hpp::pinocchio;

void displayAABB(const fcl::AABB& aabb)
{
    std::cout << "Bounding box is\n"
      << aabb.min_.transpose() << '\n'
      << aabb.max_.transpose() << std::endl;
}

BOOST_AUTO_TEST_CASE (computeAABB)
{
  DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidSimple);
  BOOST_REQUIRE(robot);

  robot->rootJoint()->lowerBounds(vector3_t::Constant(-0));
  robot->rootJoint()->upperBounds(vector3_t::Constant( 0));
  fcl::AABB aabb0 = robot->computeAABB();
  if (verbose) displayAABB(aabb0);

  robot->rootJoint()->lowerBounds(vector3_t(-1, -1, 0));
  robot->rootJoint()->upperBounds(vector3_t( 1,  1, 0));
  fcl::AABB aabb1 = robot->computeAABB();
  if (verbose) displayAABB(aabb1);

  robot->rootJoint()->lowerBounds(vector3_t(-2, -2, 0));
  robot->rootJoint()->upperBounds(vector3_t(-1, -1, 0));
  fcl::AABB aabb2 = robot->computeAABB();
  if (verbose) displayAABB(aabb2);
}
/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (unit_test_device)
{
  DevicePtr_t robot;
  LiegroupSpacePtr_t space;

  robot = unittest::makeDevice (unittest::HumanoidSimple);
  space = LiegroupSpace::createCopy(robot->configSpace());
  space->mergeVectorSpaces();
  BOOST_CHECK_EQUAL (space->name(), "SE(3)*R^26");

  space = LiegroupSpace::createCopy(robot->RnxSOnConfigSpace());
  space->mergeVectorSpaces();
  BOOST_CHECK_EQUAL (space->name(), "R^3*SO(3)*R^26");

  robot->setDimensionExtraConfigSpace(3);
  BOOST_CHECK_EQUAL(robot->numberDof(), 32+3);
  BOOST_CHECK_EQUAL(robot->configSize(), 33+3);

  space = LiegroupSpace::createCopy(robot->configSpace());
  space->mergeVectorSpaces();
  BOOST_CHECK_EQUAL (space->name(), "SE(3)*R^29");

  robot = unittest::makeDevice (unittest::CarLike);
  space = LiegroupSpace::createCopy(robot->configSpace());
  space->mergeVectorSpaces();
  BOOST_CHECK_EQUAL (space->name(), "SE(2)*R^2");

  robot = unittest::makeDevice (unittest::ManipulatorArm2);
  space = LiegroupSpace::createCopy(robot->configSpace());
  space->mergeVectorSpaces();
  BOOST_CHECK_EQUAL (space->name(), "R^19");
}
