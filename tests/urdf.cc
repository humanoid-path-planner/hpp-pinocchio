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
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>

using namespace hpp::pinocchio;

BOOST_AUTO_TEST_CASE(mimic_joint) {
  std::string urdf(
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
  DevicePtr_t robot = Device::create("test");
  urdf::loadModelFromString(robot, 0, "test", "anchor", urdf, "");
  BOOST_REQUIRE(robot);

  BOOST_REQUIRE_EQUAL(robot->jointConstraints().size(), 1);
  BOOST_CHECK_EQUAL(robot->jointConstraints()[0].multiplier, -1.);
  BOOST_CHECK_EQUAL(robot->jointConstraints()[0].offset, 0.5);
}

BOOST_AUTO_TEST_CASE(append_model) {
  DevicePtr_t robot = Device::create("test");
  FrameIndex baseFrame = 0;
  matrix3_t R;
  R << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  vector3_t t; t << 1, 2, 3;
  ::pinocchio::SE3 wMr(R, t), Id; Id.setIdentity();
  std::string urdfPath
    ("package://example-robot-data/robots/ur_description/urdf/ur5_gripper.urdf");
  std::string srdfPath
    ("package://example-robot-data/robots/ur_description/srdf/ur5_gripper.srdf");
  std::string boxUrdf(
     "<robot name='box'>"
    "  <link name='base_link'>"
    "    <inertial>"
    "      <origin xyz='0.0 0.0 0.0' rpy='0 0 0' />"
    "      <mass value='0.6'/>"
    "      <inertia ixx='0.001' ixy='0.0' ixz='0.0'"
    "	       iyy='0.001' iyz='0.0'"
    "	       izz='0.001' />"
    "    </inertial>"
    "    <visual>"
    "      <origin xyz='0 0 0' rpy='0 0 0' />"
    "      <geometry>"
    "        <box size='0.03 0.05 0.05'/>"
    "      </geometry>"
    "      <material name='white'>"
    "        <color rgba='1 1 1 1'/>"
    "      </material>"
    "    </visual>"
    "    <collision>"
    "      <origin xyz='0 0 0' rpy='0 0 0' />"
    "      <geometry>"
    "        <box size='0.03 0.05 0.05'/>"
    "      </geometry>"
    "    </collision>"
    "  </link>"
    "</robot>");
  std::string boxSrdf(
    "<robot name='box'>"
    "</robot>");

  // Load ur5
  urdf::loadModel(robot, baseFrame, "ur5", "anchor", urdfPath, srdfPath,
                  wMr);
  // append box
  urdf::loadModelFromString(robot, baseFrame, "box", "freeflyer", boxUrdf, boxSrdf, Id);

  BOOST_CHECK_EQUAL(robot->model().names[1], "ur5/shoulder_pan_joint");
  BOOST_CHECK_EQUAL(robot->model().names[2], "ur5/shoulder_lift_joint");
  BOOST_CHECK_EQUAL(robot->model().names[3], "ur5/elbow_joint");
  BOOST_CHECK_EQUAL(robot->model().names[4], "ur5/wrist_1_joint");
  BOOST_CHECK_EQUAL(robot->model().names[5], "ur5/wrist_2_joint");
  BOOST_CHECK_EQUAL(robot->model().names[6], "ur5/wrist_3_joint");
  BOOST_CHECK_EQUAL(robot->model().names[7], "box/root_joint");

  vector_t q(robot->model().nq);
  ::pinocchio::neutral(robot->model(), q);
  ::pinocchio::forwardKinematics(robot->model(), robot->data(), q);
  ::pinocchio::updateFramePlacements(robot->model(), robot->data());
  FrameIndex i(robot->model().getFrameId("ur5/base_link"));
  BOOST_CHECK_EQUAL(robot->data().oMf[i], wMr);
}
