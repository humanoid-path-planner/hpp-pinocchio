//
// Copyright (c) 2016 CNRS
// Author: Joseph Mirabel from Florent Lamiraux
//
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

#include <hpp/pinocchio/humanoid-robot.hh>

#include <boost/serialization/weak_ptr.hpp>

#include <pinocchio/serialization/eigen.hpp>

#include <hpp/util/serialization.hh>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/serialization.hh>

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
      waist_      = Joint::create (d, other.waist_     ->index());
      chest_      = Joint::create (d, other.chest_     ->index());
      leftWrist_  = Joint::create (d, other.leftWrist_ ->index());
      rightWrist_ = Joint::create (d, other.rightWrist_->index());
      leftAnkle_  = Joint::create (d, other.leftAnkle_ ->index());
      rightAnkle_ = Joint::create (d, other.rightAnkle_->index());
      gazeJoint_  = Joint::create (d, other.gazeJoint_ ->index());
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

      template<class Archive>
      void HumanoidRobot::serialize(Archive & ar, const unsigned int version)
      {
        (void) version;
        auto* har = hpp::serialization::cast(&ar);

        ar & boost::serialization::make_nvp("base", boost::serialization::base_object<Device>(*this));
        // TODO we should throw if a Device instance with name name_ and not of
        // type HumanoidRobot is found.
        bool written = (!har ||
            har->template getChildClass<Device, HumanoidRobot>(name_, false) != this);
        ar & BOOST_SERIALIZATION_NVP(written);
        if (written) {
          ar & BOOST_SERIALIZATION_NVP(weakPtr_);
          ar & BOOST_SERIALIZATION_NVP(waist_);
          ar & BOOST_SERIALIZATION_NVP(chest_);
          ar & BOOST_SERIALIZATION_NVP(leftWrist_);
          ar & BOOST_SERIALIZATION_NVP(rightWrist_);
          ar & BOOST_SERIALIZATION_NVP(leftAnkle_);
          ar & BOOST_SERIALIZATION_NVP(rightAnkle_);
          ar & BOOST_SERIALIZATION_NVP(gazeJoint_);
          ar & BOOST_SERIALIZATION_NVP(gazeOrigin_);
          ar & BOOST_SERIALIZATION_NVP(gazeDirection_);
        }
      }

      HPP_SERIALIZATION_IMPLEMENT(HumanoidRobot);
  } // namespace pinocchio
} // namespace hpp

BOOST_CLASS_EXPORT_IMPLEMENT(hpp::pinocchio::HumanoidRobot)
