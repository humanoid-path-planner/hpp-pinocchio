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

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/test/unit_test.hpp>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/algorithm/check.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <sstream>

using namespace hpp::pinocchio;

void displayAABB(const hpp::fcl::AABB& aabb) {
  BOOST_TEST_MESSAGE("Bounding box is\n"
                     << aabb.min_.transpose() << '\n'
                     << aabb.max_.transpose());
}

BOOST_AUTO_TEST_CASE(computeAABB) {
  DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidSimple);
  BOOST_REQUIRE(robot);

  robot->rootJoint()->lowerBounds(vector3_t::Constant(-0));
  robot->rootJoint()->upperBounds(vector3_t::Constant(0));
  hpp::fcl::AABB aabb0 = robot->computeAABB();
  displayAABB(aabb0);

  robot->rootJoint()->lowerBounds(vector3_t(-1, -1, 0));
  robot->rootJoint()->upperBounds(vector3_t(1, 1, 0));
  hpp::fcl::AABB aabb1 = robot->computeAABB();
  displayAABB(aabb1);

  robot->rootJoint()->lowerBounds(vector3_t(-2, -2, 0));
  robot->rootJoint()->upperBounds(vector3_t(-1, -1, 0));
  hpp::fcl::AABB aabb2 = robot->computeAABB();
  displayAABB(aabb2);
}
/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE(unit_test_device) {
  DevicePtr_t robot;
  LiegroupSpacePtr_t space;

  robot = Device::create("robot");
  BOOST_CHECK(pinocchio::checkData(robot->model(), robot->data()));

  robot = unittest::makeDevice(unittest::HumanoidSimple);
  space = LiegroupSpace::createCopy(robot->configSpace());
  space->mergeVectorSpaces();
  BOOST_CHECK_EQUAL(space->name(), "SE(3)*R^26");

  space = LiegroupSpace::createCopy(robot->RnxSOnConfigSpace());
  space->mergeVectorSpaces();
  BOOST_CHECK_EQUAL(space->name(), "R^3*SO(3)*R^26");

  robot->setDimensionExtraConfigSpace(3);
  BOOST_CHECK_EQUAL(robot->numberDof(), 32 + 3);
  BOOST_CHECK_EQUAL(robot->configSize(), 33 + 3);

  space = LiegroupSpace::createCopy(robot->configSpace());
  space->mergeVectorSpaces();
  BOOST_CHECK_EQUAL(space->name(), "SE(3)*R^29");

  BOOST_TEST_MESSAGE(*robot);
  robot->removeJoints({"rleg2_joint", "rarm1_joint"},
                      robot->neutralConfiguration());
  BOOST_TEST_MESSAGE(*robot);

  BOOST_CHECK_THROW(robot->getJointByName("rleg2_joint"), std::runtime_error);
  BOOST_CHECK_THROW(robot->getJointByName("rarm1_joint"), std::runtime_error);

  robot = unittest::makeDevice(unittest::CarLike);
  space = LiegroupSpace::createCopy(robot->configSpace());
  space->mergeVectorSpaces();
  BOOST_CHECK_EQUAL(space->name(), "SE(2)*R^2");

  robot = unittest::makeDevice(unittest::ManipulatorArm2);
  space = LiegroupSpace::createCopy(robot->configSpace());
  space->mergeVectorSpaces();
  BOOST_CHECK_EQUAL(space->name(), "R^19");
}

BOOST_AUTO_TEST_CASE(load_neutral_configuration) {
  std::string urdf(
      "<robot name='test'>"
      "<link name='base_link'/>"
      "<link name='link1'/>"
      "<joint name='joint1' type='revolute'>"
      "  <parent link='base_link'/>"
      "  <child link='link1'/>"
      "  <limit effort='30' velocity='1.0' />"
      "</joint>"
      "</robot>");
  std::string srdf(
      "<robot name='test'>"
      "<group name='all'/>"
      "<group_state name='half_sitting' group='all'>"
      "<joint name='joint1' value='1.' />"
      "</group_state>"
      "</robot>");

  DevicePtr_t device = Device::create("test");
  urdf::loadModelFromString(device, 0, "", "anchor", urdf, srdf);
  BOOST_CHECK(device);
  BOOST_CHECK_EQUAL(device->neutralConfiguration().size(),
                    device->configSize());
  BOOST_CHECK_MESSAGE(device->neutralConfiguration().isZero(1e-12),
                      "neutral configuration - wrong results");

  /*
  // TODO When neutral configuration can be read from XML string, this test
  should be updated in order to
  // read a URDF and SRDF string rather than a file in a different package.
  Eigen::VectorXd expected(device->configSize());
  expected.setOnes();

  const Model& model = device->model();
  Model::ConfigVectorMap::const_iterator half_sitting =
  model.referenceConfigurations.find ("half_sitting");
  BOOST_REQUIRE(half_sitting != model.referenceConfigurations.end());
  BOOST_CHECK_MESSAGE(half_sitting->second.isApprox (expected, 1e-12),
      "reference configuration - wrong results\ngot: "
      << half_sitting->second.transpose() << "\nexpected: "
      << expected.transpose());
      */
}

BOOST_AUTO_TEST_CASE(serialization) {
  DevicePtr_t device = unittest::makeDevice(unittest::HumanoidRomeo);
  JointPtr_t joint = Joint::create(device, 1);

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
