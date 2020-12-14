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

#include <sstream>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <pinocchio/fwd.hpp>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/check.hpp>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/serialization.hh>
static bool verbose = true;

using namespace hpp::pinocchio;

void displayAABB(const hpp::fcl::AABB& aabb)
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
  hpp::fcl::AABB aabb0 = robot->computeAABB();
  if (verbose) displayAABB(aabb0);

  robot->rootJoint()->lowerBounds(vector3_t(-1, -1, 0));
  robot->rootJoint()->upperBounds(vector3_t( 1,  1, 0));
  hpp::fcl::AABB aabb1 = robot->computeAABB();
  if (verbose) displayAABB(aabb1);

  robot->rootJoint()->lowerBounds(vector3_t(-2, -2, 0));
  robot->rootJoint()->upperBounds(vector3_t(-1, -1, 0));
  hpp::fcl::AABB aabb2 = robot->computeAABB();
  if (verbose) displayAABB(aabb2);
}
/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (unit_test_device)
{
  DevicePtr_t robot;
  LiegroupSpacePtr_t space;

  robot = Device::create ("robot");
  BOOST_CHECK(pinocchio::checkData (robot->model(), robot->data()));

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

BOOST_AUTO_TEST_CASE(load_neutral_configuration)
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
      "</robot>");
  std::string srdf (
      "<robot name='test'>"
      "<group name='all'/>"
      "<group_state name='half_sitting' group='all'>"
      "<joint name='joint1' value='1.' />"
      "</group_state>"
      "</robot>");

  DevicePtr_t device = Device::create("test");
  urdf::loadModelFromString (device, 0, "", "anchor", urdf, srdf);
  BOOST_CHECK(device);
  BOOST_CHECK_EQUAL(device->neutralConfiguration().size(),device->configSize());
  BOOST_CHECK_MESSAGE(device->neutralConfiguration().isZero(1e-12), "neutral configuration - wrong results");

  /*
  // TODO When neutral configuration can be read from XML string, this test should be updated in order to
  // read a URDF and SRDF string rather than a file in a different package.
  Eigen::VectorXd expected(device->configSize());
  expected.setOnes();

  const Model& model = device->model();
  Model::ConfigVectorMap::const_iterator half_sitting = model.referenceConfigurations.find ("half_sitting"); 
  BOOST_REQUIRE(half_sitting != model.referenceConfigurations.end());
  BOOST_CHECK_MESSAGE(half_sitting->second.isApprox (expected, 1e-12),
      "reference configuration - wrong results\ngot: "
      << half_sitting->second.transpose() << "\nexpected: "
      << expected.transpose());
      */
}

BOOST_AUTO_TEST_CASE(serialization)
{
  DevicePtr_t device = unittest::makeDevice (unittest::HumanoidRomeo);
  JointPtr_t joint = Joint::create (device, 1);

  std::stringstream ss;
  {
    hpp::serialization::xml_oarchive oa(ss);
    oa.initialize();
    oa << boost::serialization::make_nvp("device", device);
    oa << boost::serialization::make_nvp("joint", joint);
  }

  BOOST_TEST_MESSAGE(ss.str());

  DevicePtr_t device2;
  JointPtr_t joint2;
  {
    hpp::serialization::xml_iarchive ia(ss);
    ia.insert(device->name(), device.get());
    ia.initialize();
    ia >> boost::serialization::make_nvp("device", device2);
    ia >> boost::serialization::make_nvp("joint", joint2);
  }

  BOOST_CHECK(device == device2);
  BOOST_CHECK(joint2->robot() == device2);
}
