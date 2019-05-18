//
// Copyright (c) 2016 CNRS
// Author: NMansard from Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_DEVICE_DATA_HH
#define HPP_PINOCCHIO_DEVICE_DATA_HH

# include <hpp/util/debug.hh>

# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace pinocchio {
    enum Computation_t {
      JOINT_POSITION = 0x1,
      JACOBIAN       = 0x2,
      VELOCITY       = 0x4,
      ACCELERATION   = 0x8,
      COM            = 0xf,
      COMPUTE_ALL    = 0Xffff
    };

    /// Struct containing the Device data.
    /// Users normally do not need to access its attributes.
    struct DeviceData
    {
      DeviceData ();
      DeviceData (const DeviceData& other);

      inline void invalidate () { upToDate_ = false; frameUpToDate_ = false; geomUpToDate_ = false; }

      void computeForwardKinematics (const ModelPtr_t& m);
      void computeFramesForwardKinematics (const ModelPtr_t& m);
      void updateGeometryPlacements (const ModelPtr_t& m, const GeomModelPtr_t& gm);

      // Pinocchio objects
      DataPtr_t data_;
      GeomDataPtr_t geomData_;

      Configuration_t currentConfiguration_;
      vector_t currentVelocity_;
      vector_t currentAcceleration_;
      bool upToDate_, frameUpToDate_, geomUpToDate_;
      Computation_t computationFlag_;
      DeviceWkPtr_t devicePtr_;

      /// Temporary variable to avoid dynamic allocation
      Configuration_t modelConf_;
      /// Pool of joint jacobians
      std::vector<JointJacobian_t> jointJacobians_;
    }; // struct DeviceData
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_DEVICE_DATA_HH
