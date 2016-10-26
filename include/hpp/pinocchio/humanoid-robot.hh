//
// Copyright (c) 2016 CNRS
// Author: Joseph Mirabel from Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_HUMANOID_ROBOT_HH
#define HPP_PINOCCHIO_HUMANOID_ROBOT_HH

# include <iostream>
# include <vector>

# include <hpp/pinocchio/device.hh>
# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/config.hh>

namespace hpp {
  namespace pinocchio {
    /// \brief Humanoid robot

    class HPP_PINOCCHIO_DLLAPI HumanoidRobot : public Device
    {
    public:
      /// \name Construction, copy and destruction
      /// @{
      virtual ~HumanoidRobot ();

      /// \brief Clone as a HumanoidRobot
      virtual DevicePtr_t clone () const;

      ///
      /// @}
      ///

      /// \brief Creation of a new device
      /// \return a shared pointer to the new device
      /// \param name Name of the device (is passed to CkkpDeviceComponent)
      static HumanoidRobotPtr_t create (const std::string& name);

      /// \brief Get Joint corresponding to the waist.
      JointPtr_t waist() const;

      /// Set waist joint
      void waist (const JointPtr_t& joint);

      /// \brief Get Joint corresponding to the chest.
      JointPtr_t chest() const;

      /// Set chest joint
      void chest (const JointPtr_t& joint);

      /// \brief Get Joint corresponding to the left wrist.
      JointPtr_t leftWrist() const;

      /// Set left wrist
      void leftWrist (const JointPtr_t& joint);

      /// \brief Get Joint corresponding to the right wrist.
      JointPtr_t rightWrist() const;

      /// Set right wrist
      void rightWrist (const JointPtr_t& joint);

      /// \brief Get Joint corresponding to the left ankle.
      JointPtr_t leftAnkle() const;

      /// Set letf ankle
      void leftAnkle (const JointPtr_t& joint);

      /// \brief Get Joint corresponding to the right ankle.
      JointPtr_t rightAnkle() const;

      /// Set right ankle
      void rightAnkle (const JointPtr_t& joint);

      /// \brief Get gaze joint
      JointPtr_t gazeJoint() const;

      /// Set gaze joint
      void gazeJoint (const JointPtr_t& joint);

      /// Set gaze parameters
      void gaze (const vector3_t& origin, const vector3_t& dir)
      {
	gazeOrigin_ = origin;
	gazeDirection_ = dir;
      }
    protected:
      /// \brief Constructor
      HumanoidRobot (const std::string& name);

      HumanoidRobot (const HumanoidRobot& other);

      ///
      /// \brief Initialization.
      ///
      void init (const HumanoidRobotWkPtr_t& weakPtr);

      void initCopy (const HumanoidRobotWkPtr_t& weakPtr, const HumanoidRobot& other);

    private:
      HumanoidRobotWkPtr_t weakPtr_;
      JointPtr_t waist_;
      JointPtr_t chest_;
      JointPtr_t leftWrist_;
      JointPtr_t rightWrist_;
      JointPtr_t leftAnkle_;
      JointPtr_t rightAnkle_;
      JointPtr_t gazeJoint_;
      vector3_t gazeOrigin_;
      vector3_t gazeDirection_;
    }; // class HumanoidRobot
  } // namespace pinocchio
} // namespace hpp
#endif // HPP_PINOCCHIO_HUMANOID_ROBOT_HH
