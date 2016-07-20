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

# include <hpp/fcl/collision_object.h>
# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/fake-container.hh>
# include <pinocchio/multibody/fwd.hpp>

namespace se3
{
  struct GeometryObject;
}

namespace fcl {
  HPP_PREDEF_CLASS (CollisionObject);
  HPP_PREDEF_CLASS (CollisionGeometry);
} // namespace fcl

namespace hpp {
  namespace pinocchio {
    /// Specialization of fcl::CollisionObject to add a name to objects
    ///
    /// Objects moved by a robot joint. They can collide each other and
    /// distance computation can be computed between them.
    struct HPP_PINOCCHIO_DLLAPI CollisionObject {

      typedef se3::JointIndex JointIndex;
      typedef se3::GeomIndex GeomIndex;
      enum InOutType { INNER, OUTER };

      typedef std::vector<GeomIndex> GeomIndexList;
      typedef std::map < se3::JointIndex, GeomIndexList > ObjectVec_t;

      //DREPEC /// Create collision object and return shared pointer
      //DREPEC static CollisionObjectPtr_t create (fcl::CollisionObjectPtr_t object,
      //DREPEC 					  const std::string& name);
      //DREPEC /// Create collision object and return shared pointer
      //DREPEC static CollisionObjectPtr_t create (fcl::CollisionGeometryPtr_t geometry,
      //DREPEC 					  const Transform3f& position,
      //DREPEC 					  const std::string& name);
      //DREPEC /// Clone object and attach to given joint.
      //DREPEC CollisionObjectPtr_t clone (const JointPtr_t& joint) const;

      /// Construction from inner/outer list, using a joint index as reference.
      /// \param geom: index of the object in either inner or outer objects of joint <joint>.
      CollisionObject( DevicePtr_t device, 
                       const JointIndex joint,
                       const GeomIndex geom,
                       const InOutType inout = INNER );

      /// Construction from global collision-object list. The joint ID is recovered from
      /// the collision object in Pinocchio and InOut is set to INNER. The geomIndexInJoint
      /// is not defined (geomIndexInJointSet = false)
      CollisionObject( DevicePtr_t device, 
                       const GeomIndex geom );

      const std::string& name () const;

      /// Access to pinocchio object
      const se3::GeometryObject & pinocchio () const;
      se3::GeometryObject &       pinocchio ();
      
      /// Access to fcl object
      fclConstCollisionObjectPtr_t fcl () const ;
      fclCollisionObjectPtr_t fcl () ;

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

      //DEPREC /// \name Construction, destruction and copy
      //DEPREC /// \{
      //DEPREC 
      //DEPREC /// Wrap fcl collision object at identity position
      //DEPREC explicit CollisionObject (fcl::CollisionObjectPtr_t object,
      //DEPREC 				const std::string& name) :
      //DEPREC 	object_ (object), joint_ (0), name_ (name), weakPtr_ ()
      //DEPREC 	{
      //DEPREC 	  positionInJointFrame_.setIdentity ();
      //DEPREC 	}
      //DEPREC /// Wrap fcl collision object and put at given position
      //DEPREC explicit CollisionObject (const fcl::CollisionGeometryPtr_t geometry,
      //DEPREC 				const Transform3f& position,
      //DEPREC 				const std::string& name) :
      //DEPREC 	object_ (new fcl::CollisionObject (geometry, position)),
      //DEPREC 	joint_ (0), name_ (name), weakPtr_ ()
      //DEPREC 	{
      //DEPREC 	  positionInJointFrame_ = position;
      //DEPREC 	}
      //DEPREC /// Copy constructor
      //DEPREC explicit CollisionObject (const CollisionObject& object) :
      //DEPREC object_ (new fcl::CollisionObject (object.fcl()->collisionGeometry(),
      //DEPREC                                object.fcl()->getTransform())),
      //DEPREC positionInJointFrame_ (object.positionInJointFrame_),
      //DEPREC joint_ (0x0),
      //DEPREC name_ (object.name_),
      //DEPREC weakPtr_ ()
      //DEPREC   {
      //DEPREC   }
      //DEPREC 
      //DEPREC /// \}
      //DEPREC void init (const CollisionObjectWkPtr_t& self)
      //DEPREC {
      //DEPREC 	weakPtr_ = self;
      //DEPREC }

    private:
      DevicePtr_t devicePtr;
      JointIndex  jointIndex;
      GeomIndex   geomInJointIndex;     // Index in joint list.
      bool        geomInJointIndexSet;  // True if geomInJointIndex is set.
      GeomIndex   geomInModelIndex;     // Index in global model list.
      InOutType   inOutType;            // Object in Inner or Outer object list.

      //DEPREC fcl::CollisionObjectPtr_t object_;
      //DEPREC fcl::Transform3f positionInJointFrame_;
      //DEPREC JointPtr_t joint_;
      //DEPREC std::string name_;
      //DEPREC CollisionObjectWkPtr_t weakPtr_;
    }; // class CollisionObject


    typedef boost::shared_ptr<CollisionObject> CollisionObjectPtr_t;
    typedef boost::shared_ptr<const CollisionObject> CollisionObjectConstPtr_t;

    /* --- CONTAINER -------------------------------------------------------- */
    struct ObjectVector 
      : public FakeContainer<CollisionObjectPtr_t,CollisionObjectConstPtr_t>
    {
      typedef se3::JointIndex JointIndex;
      typedef CollisionObject::InOutType InOutType;

      JointIndex jointIndex;
      InOutType inOutType;

      ObjectVector(DevicePtr_t device,const JointIndex i,CollisionObject::InOutType inout)
        : FakeContainer<CollisionObjectPtr_t,CollisionObjectConstPtr_t>(device)
        , jointIndex(i), inOutType(inout) {}
      ObjectVector() {}

      virtual CollisionObjectPtr_t at(const size_type i) ;
      virtual CollisionObjectConstPtr_t at(const size_type i) const ;
      virtual size_type size() const ;

      void selfAssert(size_type i = 0) const;
    };

    typedef ObjectVector ObjectVector_t;

  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_COLLISION_OBJECT_HH
