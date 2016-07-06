//
// Copyright (c) 2016 CNRS
// Author: NMansard
//
//
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

#define BOOST_TEST_MODULE tdevice

#include <boost/test/unit_test.hpp>

#include <hpp/model/device.hh>
#include <hpp/model/urdf/util.hh>

#include <hpp/pinocchio/device.hh>
#include <pinocchio/multibody/parser/urdf.hpp>

namespace hpp {
  namespace model {
    DevicePtr_t
    robotFromUrdf( const std::string filename, const std::string rootType="freeflyer" )
    {
      // Create the robot object.
      DevicePtr_t robot  = hpp::model::Device::create("robot");;
      // Build robot model from URDF.
      urdf::Parser urdfParser (rootType, robot);
      urdfParser.parse (filename);
      hppDout (notice, "Finished parsing URDF file.");
      return robot;
    }
  } // namespace hpp
} // namespace model


BOOST_AUTO_TEST_CASE (tdevice)
{
  std::string urdfFilename = "file:///home/nmansard/src/pinocchio/hpp/hpp-model-urdf/romeo.urdf";
  hpp::model::DevicePtr_t robot  = hpp::model::robotFromUrdf(urdfFilename);

  hpp::pinocchio::DevicePtr_t pinocchio = hpp::pinocchio::Device::create("pinocchio");
  hpp::pinocchio::ModelPtr_t model( new se3::Model() );
  *model = se3::urdf::buildModel(urdfFilename,se3::JointModelFreeFlyer());
  pinocchio->model(model);
  pinocchio->createData();
  
  std::ofstream log ("./display-robot.log");
  log << *(robot.get ()) << std::endl;
}
