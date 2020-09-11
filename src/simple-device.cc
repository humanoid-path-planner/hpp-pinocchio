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

# include <hpp/pinocchio/simple-device.hh>

# include <pinocchio/parsers/sample-models.hpp>

# include <hpp/pinocchio/device.hh>
# include <hpp/pinocchio/humanoid-robot.hh>
# include <hpp/pinocchio/urdf/util.hh>
# include <hpp/pinocchio/joint-collection.hh>

namespace hpp {
  namespace pinocchio {
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void humanoidRandom(::pinocchio::ModelTpl<Scalar,Options,JointCollectionTpl> & model)
    {
      using ::pinocchio::buildModels::details::addJointAndBody;
      static const SE3 Id = SE3::Identity();
      typedef JointCollectionTpl<Scalar, Options> JC;

      // root
      addJointAndBody(model, typename JC::JointModelFreeFlyer(), "universe", "root", Id);
      model.lowerPositionLimit.template segment<4>(3).fill(-1.);
      model.upperPositionLimit.template segment<4>(3).fill( 1.);

      // lleg
      addJointAndBody(model,typename JC::JointModelRX(),"root_joint","lleg1");
      addJointAndBody(model,typename JC::JointModelRY(),"lleg1_joint","lleg2");
      addJointAndBody(model,typename JC::JointModelRZ(),"lleg2_joint","lleg3");
      addJointAndBody(model,typename JC::JointModelRY(),"lleg3_joint","lleg4");
      addJointAndBody(model,typename JC::JointModelRY(),"lleg4_joint","lleg5");
      addJointAndBody(model,typename JC::JointModelRX(),"lleg5_joint","lleg6");

      // rleg
      addJointAndBody(model,typename JC::JointModelRX(),"root_joint","rleg1");
      addJointAndBody(model,typename JC::JointModelRY(),"rleg1_joint","rleg2");
      addJointAndBody(model,typename JC::JointModelRZ(),"rleg2_joint","rleg3");
      addJointAndBody(model,typename JC::JointModelRY(),"rleg3_joint","rleg4");
      addJointAndBody(model,typename JC::JointModelRY(),"rleg4_joint","rleg5");
      addJointAndBody(model,typename JC::JointModelRX(),"rleg5_joint","rleg6");

      // trunc
      addJointAndBody(model,typename JC::JointModelRY(),"root_joint","torso1");
      addJointAndBody(model,typename JC::JointModelRZ(),"torso1_joint","chest");

      // rarm
      addJointAndBody(model,typename JC::JointModelRX(),"chest_joint","rarm1");
      addJointAndBody(model,typename JC::JointModelRY(),"rarm1_joint","rarm2");
      addJointAndBody(model,typename JC::JointModelRZ(),"rarm2_joint","rarm3");
      addJointAndBody(model,typename JC::JointModelRY(),"rarm3_joint","rarm4");
      addJointAndBody(model,typename JC::JointModelRY(),"rarm4_joint","rarm5");
      addJointAndBody(model,typename JC::JointModelRX(),"rarm5_joint","rarm6");

      // larm
      addJointAndBody(model,typename JC::JointModelRX(),"chest_joint","larm1");
      addJointAndBody(model,typename JC::JointModelRY(),"larm1_joint","larm2");
      addJointAndBody(model,typename JC::JointModelRZ(),"larm2_joint","larm3");
      addJointAndBody(model,typename JC::JointModelRY(),"larm3_joint","larm4");
      addJointAndBody(model,typename JC::JointModelRY(),"larm4_joint","larm5");
      addJointAndBody(model,typename JC::JointModelRX(),"larm5_joint","larm6");

    }

    DevicePtr_t humanoidSimple(
        const std::string& name,
        bool usingFF,
        Computation_t compFlags)
    {
      if (!usingFF)
        throw std::invalid_argument ("Humanoid simple without freefloating base is not supported anymore.");
      return humanoidSimple (name, compFlags);
    }

    DevicePtr_t humanoidSimple(
        const std::string& name,
        Computation_t compFlags)
    {
      DevicePtr_t robot = Device::create (name);
      humanoidRandom(robot->model());
      robot->createData();
      robot->createGeomData();
      robot->controlComputation(compFlags);
      robot->currentConfiguration(robot->neutralConfiguration());
      robot->computeForwardKinematics();

      const value_type max =  std::numeric_limits<value_type>::max();
      const value_type min = -std::numeric_limits<value_type>::max();
      robot->model().lowerPositionLimit.segment<3>(0).setConstant(min);
      robot->model().upperPositionLimit.segment<3>(0).setConstant(max);
      robot->model().lowerPositionLimit.segment<4>(3).setConstant(-1.01);
      robot->model().upperPositionLimit.segment<4>(3).setConstant( 1.01);
      return robot;
    }

    namespace unittest {
      DevicePtr_t makeDevice (TestDeviceType type,
                              const std::string& prefix)
      {
        DevicePtr_t robot;
        HumanoidRobotPtr_t hrobot;
        (void)prefix;
        switch (type) {
          case CarLike:
            robot  = Device::create("carlike");
            urdf::loadRobotModel (robot, 0, prefix, "planar",
                "hpp_environments", "buggy", "", "");
            robot->model().lowerPositionLimit.head<2>().setConstant(-1);
            robot->model().upperPositionLimit.head<2>().setOnes();
            return robot;
          case ManipulatorArm2:
            robot  = Device::create("arm");
            urdf::loadRobotModel (robot, 0, prefix, "anchor",
                "hpp_environments", "tests/baxter",
                "_simple", "_simple");
            return robot;
          case HumanoidRomeo:
            hrobot  = HumanoidRobot::create("romeo");
            urdf::loadRobotModel (hrobot, 0, prefix, "freeflyer",
                                  "example-robot-data/robots/romeo_description",
                                  "romeo", "_small", "_small");
            urdf::setupHumanoidRobot (hrobot, prefix);
            hrobot->model().lowerPositionLimit.head<3>().setConstant(-1);
            hrobot->model().upperPositionLimit.head<3>().setOnes();
            return hrobot;
          case HumanoidSimple:
            robot = humanoidSimple ("simple-humanoid",
                (Computation_t) (JOINT_POSITION | JACOBIAN));
            robot->model().lowerPositionLimit.head<3>().setConstant(-1);
            robot->model().upperPositionLimit.head<3>().setOnes();
            return robot;
          default:
            throw std::invalid_argument("Unknown robot type.");
        }
      }
    }
  } // namespace pinocchio
} // namespace hpp
