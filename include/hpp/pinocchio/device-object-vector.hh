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
# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/deprecated.hh>
# include <hpp/pinocchio/fake-container.hh>

namespace hpp {
  namespace pinocchio {

    /// Iterator over all inner objects of a Device.
    /// \deprecated Use Device::nbObjects and Device::objectAt
    struct  DeviceObjectVector
      : public FakeContainer<CollisionObjectPtr_t,CollisionObjectConstPtr_t>
    {
      DeviceObjectVector(DeviceWkPtr_t device)
      : FakeContainer<CollisionObjectPtr_t,CollisionObjectConstPtr_t>(device) {}
      DeviceObjectVector() {}

      virtual CollisionObjectPtr_t at(const size_type i) ;
      virtual CollisionObjectConstPtr_t at(const size_type i) const ;
      virtual size_type size() const ;

      void selfAssert(size_type i = 0) const;
    } HPP_PINOCCHIO_DEPRECATED; // struct DeviceObjectVector

    /* --- CONTAINER -------------------------------------------------------- */
    struct ObjectVector 
      : public FakeContainer<CollisionObjectPtr_t,CollisionObjectConstPtr_t>
    {
      typedef se3::JointIndex JointIndex;

      JointIndex jointIndex;
      InOutType inOutType;

      ObjectVector(DeviceWkPtr_t device,const JointIndex i, InOutType inout)
        : FakeContainer<CollisionObjectPtr_t,CollisionObjectConstPtr_t>(device)
        , jointIndex(i), inOutType(inout) {}
      ObjectVector() {}

      virtual CollisionObjectPtr_t at(const size_type i) ;
      virtual CollisionObjectConstPtr_t at(const size_type i) const ;
      virtual size_type size() const ;

      void selfAssert(size_type i = 0) const;

      private:
      typedef std::vector<se3::GeomIndex> GeomIndexList;
      const GeomIndexList & geometries() const;
    } HPP_PINOCCHIO_DEPRECATED;

    /** Fake std::vector<Joint>, used to comply with the actual structure of hpp::model.
     *
     * You can use it for the following loop:
     *       for (JointVector_t::const_iterator it = jv.begin (); 
     *               it != jv.end (); ++it) 
     *          cout << (*it)->name;
     */
    struct JointVector
      : public FakeContainer<JointPtr_t,JointConstPtr_t>
    {
      JointVector(DeviceWkPtr_t device) : FakeContainer<JointPtr_t,JointConstPtr_t>(device) {}
      JointVector() {}
      virtual ~JointVector() {}

      virtual JointPtr_t at(const size_type i) ;
      virtual JointConstPtr_t at(const size_type i) const ;
      virtual size_type size() const ;
      virtual size_type iend() const ;

      void selfAssert(size_type i = 0) const;
    };
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_OBJECT_ITERATOR_HH
