//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel from Florent Lamiraux
//
//
// This file is part of hpp-model.
// hpp-model is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-model is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-model. If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/pinocchio/gripper.hh>

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>

namespace hpp {
  namespace pinocchio {
    Gripper::Gripper (const std::string& name, const DeviceWkPtr_t& device) :
      name_ (name),
      device_ (device),
      clearance_ (0)
    {
      DevicePtr_t d = this->device();
      fid_ = d->model().getFrameId (name);
      // TODO as joint_ keeps a shared pointer to the device, the device will
      // never be deleted.
      joint_ = JointPtr_t (
          new Joint(d, d->model().frames[fid_].parent));
    }

    const Transform3f& Gripper::objectPositionInJoint () const
    {
      return device()->model().frames[fid_].placement;
    }

    GripperPtr_t Gripper::clone () const
    {
      return Gripper::create (name_, device_);
    }

    std::ostream& Gripper::print (std::ostream& os) const
    {
      os << "name :" << name () << std::endl;
      os << "Position in joint :" << objectPositionInJoint ();
      os << "Joint :" << joint ()->name () << std::endl;
      os << std::endl;
      return os;
    }

    std::ostream& operator<< (std::ostream& os, const Gripper& gripper)
    {
      return gripper.print (os);
    }

    DevicePtr_t Gripper::device() const
    {
      DevicePtr_t d = device_.lock();
      assert (d);
      return d;
    }
  } // namespace pinocchio
} // namespace hpp
