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

#include <hpp/pinocchio/humanoid-robot.hh>

#include <hpp/pinocchio/joint.hh>

namespace hpp {
  namespace pinocchio {

    HumanoidRobot::HumanoidRobot (const std::string& name)
      : Device (name), weakPtr_ (),
      gazeOrigin_ (0, 0, 0),
      gazeDirection_ (1, 0, 0)
    {
    }

    // ========================================================================

    HumanoidRobot::HumanoidRobot (const HumanoidRobot& other)
      : Device (other), weakPtr_ ()
      , gazeOrigin_ (other.gazeOrigin_)
      , gazeDirection_ (other.gazeDirection_)
    {
    }

    // ========================================================================

    HumanoidRobot::~HumanoidRobot ()
    {
    }

    // ========================================================================

    HumanoidRobotPtr_t HumanoidRobot::create (const std::string& name)
    {
      HumanoidRobot *hppHumanoidRobot = new HumanoidRobot (name);
      HumanoidRobotPtr_t hppHumanoidRobotPtr_t (hppHumanoidRobot);

      hppHumanoidRobot->init (hppHumanoidRobotPtr_t);
      return hppHumanoidRobotPtr_t;
    }

    // ========================================================================

    DevicePtr_t HumanoidRobot::clone () const
    {
      HumanoidRobot *ptr = new HumanoidRobot (*this);
      HumanoidRobotPtr_t shPtr (ptr);
      ptr->initCopy (shPtr, *this);
      return shPtr;
    }

    // ========================================================================

    void HumanoidRobot::init(const HumanoidRobotWkPtr_t& weakPtr)
    {
      Device::init (weakPtr);
      weakPtr_ = weakPtr;
    }

    // ========================================================================

    void HumanoidRobot::initCopy (const HumanoidRobotWkPtr_t& weakPtr, const HumanoidRobot& other)
    {
      // Cannot call HumanoidRobot::init because Device::init would be called
      // twice.
      weakPtr_ = weakPtr;
      Device::initCopy (weakPtr, other);
      // TODO the HumanoidRobot will be never be deleted as these joints have
      // a shared pointer to the device.
      DevicePtr_t d = weakPtr_.lock();
      waist_      = JointPtr_t (new Joint (d, other.waist_     ->index()));
      chest_      = JointPtr_t (new Joint (d, other.chest_     ->index()));
      leftWrist_  = JointPtr_t (new Joint (d, other.leftWrist_ ->index()));
      rightWrist_ = JointPtr_t (new Joint (d, other.rightWrist_->index()));
      leftAnkle_  = JointPtr_t (new Joint (d, other.leftAnkle_ ->index()));
      rightAnkle_ = JointPtr_t (new Joint (d, other.rightAnkle_->index()));
      gazeJoint_  = JointPtr_t (new Joint (d, other.gazeJoint_ ->index()));
    }

    // ========================================================================

      /// \brief Get Joint corresponding to the waist.
      JointPtr_t HumanoidRobot::waist() const
      {
	return waist_;
      }

      /// Set waist joint
      void HumanoidRobot::waist (const JointPtr_t& joint)
      {
	waist_ = joint;
      }

      /// \brief Get Joint corresponding to the chest.
      JointPtr_t HumanoidRobot::chest() const
      {
	return chest_;
      }

      /// Set chest joint
      void HumanoidRobot::chest (const JointPtr_t& joint)
      {
	chest_ = joint;
      }

      /// \brief Get Joint corresponding to the left wrist.
      JointPtr_t HumanoidRobot::leftWrist() const
      {
	return leftWrist_;
      }

      /// Set left wrist
      void HumanoidRobot::leftWrist (const JointPtr_t& joint)
      {
	leftWrist_ = joint;
      }

      /// \brief Get Joint corresponding to the right wrist.
      JointPtr_t HumanoidRobot::rightWrist() const
      {
	return rightWrist_;
      }

      /// Set right wrist
      void HumanoidRobot::rightWrist (const JointPtr_t& joint)
      {
	rightWrist_ = joint;
      }

      /// \brief Get Joint corresponding to the left ankle.
      JointPtr_t HumanoidRobot::leftAnkle() const
      {
	return leftAnkle_;
      }

      /// Set letf ankle
      void HumanoidRobot::leftAnkle (const JointPtr_t& joint)
      {
	leftAnkle_ = joint;
      }

      /// \brief Get Joint corresponding to the right ankle.
      JointPtr_t HumanoidRobot::rightAnkle() const
      {
	return rightAnkle_;
      }

      /// Set right ankle
      void HumanoidRobot::rightAnkle (const JointPtr_t& joint)
      {
	rightAnkle_ = joint;
      }

      /// \brief Get gaze joint
      JointPtr_t HumanoidRobot::gazeJoint() const
      {
	return gazeJoint_;
      }

      /// Set gaze joint
      void HumanoidRobot::gazeJoint (const JointPtr_t& joint)
      {
	gazeJoint_ = joint;
      }

  } // namespace pinocchio
} // namespace hpp
