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

#include <hpp/model/gripper.hh>

namespace hpp {
  namespace model {
    GripperPtr_t Gripper::clone () const
    {
      GripperPtr_t self = weakPtr_.lock ();
      return Gripper::create (self->name (),self->joint (),
          self->objectPositionInJoint ());
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

  } // namespace model
} // namespace hpp
