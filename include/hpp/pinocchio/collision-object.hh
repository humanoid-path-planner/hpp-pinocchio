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

#ifndef HPP_PINOCCHIO_COLLISION_OBJECT_HH
# define HPP_PINOCCHIO_COLLISION_OBJECT_HH

#include <map>

# include <pinocchio/multibody/fwd.hpp>

# include <hpp/pinocchio/deprecated.hh>
# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>

namespace pinocchio
{
  struct GeometryObject;
}

namespace hpp {
  namespace pinocchio {
    typedef ::pinocchio::GeometryObject GeometryObject;

    /// Specialization of fcl::CollisionObject to add a name to objects
    ///
    /// Objects moved by a robot joint. They can collide each other and
    /// distance computation can be computed between them.
    class HPP_PINOCCHIO_DLLAPI CollisionObject {
    public:

      typedef std::vector<GeomIndex> GeomIndexList;
      typedef std::map < JointIndex, GeomIndexList > ObjectVec_t;

      /// Constructor for object of the device.
      CollisionObject( DevicePtr_t device,
                       const GeomIndex geom );

      /// Constructor for obstacles (object attached to the environment)
      /// It is not attached to a Device.
      CollisionObject( GeomModelPtr_t geomModel,
                       GeomDataPtr_t  geomData,
                       const GeomIndex geom );

      const std::string& name () const;

      /// Access to pinocchio object
      const GeometryObject & pinocchio () const;
      GeometryObject &       pinocchio ();
      
      /// Access to fcl object
      FclConstCollisionObjectPtr_t fcl (const GeomData& data) const;
      FclCollisionObjectPtr_t      fcl (      GeomData& data) const;
      FclConstCollisionObjectPtr_t fcl () const ;
      FclCollisionObjectPtr_t      fcl ();

      /// Returns the FCL collision object.
      /// \param d Ignored if this object represents an obstacle.
      ///          Otherwise, a DeviceData for the internal device.
      FclCollisionObjectPtr_t      fcl (DeviceData& d) const;

      /// Get joint index
      const JointIndex& jointIndex () const { return jointIndex_; }

      /// Get joint
      JointPtr_t joint () ;
      JointConstPtr_t joint () const;

      /// Return the position in the joint frame
      const Transform3f& positionInJointFrame () const;

      /// Return transform of the fcl object
      /// \warning If joint linked object -as a robot body- and the robot is
      /// manually moved, this will return the non-update transform.
      /// \note If object is not attached to a joint, use move() to update
      /// transform between hpp and fcl.
      const fcl::Transform3f& getFclTransform () const;
      const Transform3f&      getTransform () const;
      const Transform3f&      getTransform (DeviceData& d) const;

      const GeomIndex& indexInModel () const
      {
        return geomInModelIndex;
      }

      /// Move object to given position
      /// \note This method should only be executed on objects not attached
      /// to a robot body (ie attached to the "universe", joint 0). This statement
      /// is asserted.
      void move (const Transform3f& position);

    protected:

      /// Assert that the members of the struct are valid (no null pointer, etc).
      void selfAssert() const;

    private:
      GeomData& geomData() const;
      GeomData& geomData(DeviceData& d) const;

      DevicePtr_t devicePtr;
      GeomModelPtr_t geomModel_;
      // If geomData_ is not null when the object is part of the environment.
      // Otherwise, it is null.
      GeomDataPtr_t  geomData_;
      JointIndex  jointIndex_;
      GeomIndex   geomInModelIndex;     // Index in global model list.
    }; // class CollisionObject
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_COLLISION_OBJECT_HH
