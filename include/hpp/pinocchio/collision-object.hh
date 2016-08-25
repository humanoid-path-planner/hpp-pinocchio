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

namespace se3
{
  struct GeometryObject;
}

namespace hpp {
  namespace pinocchio {
    /// Specialization of fcl::CollisionObject to add a name to objects
    ///
    /// Objects moved by a robot joint. They can collide each other and
    /// distance computation can be computed between them.
    struct HPP_PINOCCHIO_DLLAPI CollisionObject {

      typedef std::vector<GeomIndex> GeomIndexList;
      typedef std::map < JointIndex, GeomIndexList > ObjectVec_t;

      /// Construction from inner/outer list, using a joint index as reference.
      /// \param geom: index of the object in either inner or outer objects of joint <joint>.
      CollisionObject( DevicePtr_t device,
                       const JointIndex joint,
                       const GeomIndex geom,
                       const InOutType inout = INNER ) HPP_PINOCCHIO_DEPRECATED;

      /// Construction from global collision-object list. The joint ID is recovered from
      /// the collision object in Pinocchio and InOut is set to INNER. The geomIndexInJoint
      /// is not defined (geomIndexInJointSet = false)
      CollisionObject( DevicePtr_t device,
                       const GeomIndex geom );

      CollisionObject( GeomModelPtr_t geomModel,
                       GeomDataPtr_t  geomData,
                       const GeomIndex geom );

      const std::string& name () const;

      /// Access to pinocchio object
      const se3::GeometryObject & pinocchio () const;
      se3::GeometryObject &       pinocchio ();
      
      /// Access to fcl object
      FclConstCollisionObjectPtr_t fcl (const GeomData& data) const;
      FclCollisionObjectPtr_t      fcl (      GeomData& data) const;
      FclConstCollisionObjectPtr_t fcl () const ;
      FclCollisionObjectPtr_t      fcl ();

      /// Get joint index
      const JointIndex& jointIndex () const { return jointIndex_; }

      /// Get joint
      JointPtr_t joint () ;
      JointConstPtr_t joint () const;

      //DEPREC /// Set joint
      //DEPREC void joint (const JointPtr_t joint);

      /// Return the position in the joint frame
      const Transform3f& positionInJointFrame () const;

      /// Return transform of the fcl object
      /// \warning If joint linked object -as a robot body- and the robot is
      /// manually moved, this will return the non-update transform.
      /// \note If object is not attached to a joint, use move() to update
      /// transform between hpp and fcl.
      const fcl::Transform3f& getFclTransform () const;
      const Transform3f&      getTransform () const;

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

      /// Get the reference to INNER|OUTER object container (marginally used).
      ObjectVec_t &       objectVec();
      const ObjectVec_t & objectVec() const;

    private:
      DevicePtr_t devicePtr;
      GeomModelPtr_t geomModel_;
      GeomDataPtr_t  geomData_;
      JointIndex  jointIndex_;
      GeomIndex   geomInModelIndex;     // Index in global model list.
      InOutType   inOutType;            // Object in Inner or Outer object list.
    }; // class CollisionObject
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_COLLISION_OBJECT_HH
