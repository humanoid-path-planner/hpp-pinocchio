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

namespace hpp {
  namespace pinocchio {
    DevicePtr_t humanoidSimple(
        const std::string& name,
        bool usingFF,
        Device::Computation_t compFlags)
    {
      DevicePtr_t robot = Device::create (name);
      se3::buildModels::humanoidSimple(robot->model(), usingFF);
      robot->createData();
      robot->createGeomData();
      robot->controlComputation(compFlags);
      robot->currentConfiguration(robot->neutralConfiguration());
      robot->computeForwardKinematics();

      const value_type max = std::numeric_limits<value_type>::max();
      const value_type min = std::numeric_limits<value_type>::max();
      robot->model().lowerPositionLimit.segment<3>(0).setConstant(min);
      robot->model().upperPositionLimit.segment<3>(0).setConstant(max);
      robot->model().lowerPositionLimit.segment<4>(3).setConstant(-1.01);
      robot->model().upperPositionLimit.segment<4>(3).setConstant( 1.01);
      return robot;
    }
  } // namespace pinocchio
} // namespace hpp
