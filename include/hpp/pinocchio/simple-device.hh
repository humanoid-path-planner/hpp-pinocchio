// Copyright (c) 2016, Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_SIMPLE_DEVICE_HH
# define HPP_PINOCCHIO_SIMPLE_DEVICE_HH

# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/device.hh>

namespace hpp {
  namespace pinocchio {
    DevicePtr_t humanoidSimple(
        const std::string& name = "humanoidSimple",
        bool usingFF = true,
        Computation_t compFlags = (Computation_t) (JOINT_POSITION | JACOBIAN)
        );

    DevicePtr_t humanoidSimple(
        const std::string& name = "humanoidSimple",
        Computation_t compFlags = (Computation_t) (JOINT_POSITION | JACOBIAN)
        );

    namespace unittest {
      enum TestDeviceType {
        HumanoidRomeo,
        HumanoidSimple,
        CarLike,
        ManipulatorArm2
      };

      DevicePtr_t makeDevice (TestDeviceType type,
                              const std::string& prefix = "");
    }
  } // namespace pinocchio
} // namespace hpp
#endif // HPP_PINOCCHIO_SIMPLE_DEVICE_HH
