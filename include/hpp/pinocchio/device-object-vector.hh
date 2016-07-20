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

#ifndef HPP_PINOCCHIO_OBJECT_ITERATOR_HH
#define HPP_PINOCCHIO_OBJECT_ITERATOR_HH

# include <vector>
# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/fake-container.hh>

namespace hpp {
  namespace pinocchio {

    /// Iterator over all inner objects of a Device.
    struct  DeviceObjectVector
      : public FakeContainer<CollisionObjectPtr_t,CollisionObjectConstPtr_t>
    {
      DeviceObjectVector(DevicePtr_t device)
      : FakeContainer<CollisionObjectPtr_t,CollisionObjectConstPtr_t>(device) {}
      DeviceObjectVector() {}

      virtual CollisionObjectPtr_t at(const size_type i) ;
      virtual CollisionObjectConstPtr_t at(const size_type i) const ;
      virtual size_type size() const ;

      void selfAssert(size_type i = 0) const;
    }; // struct DeviceObjectVector
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_OBJECT_ITERATOR_HH
