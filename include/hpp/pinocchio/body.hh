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

#ifndef HPP_PINOCCHIO_BODY_HH
# define HPP_PINOCCHIO_BODY_HH

# include <hpp/util/pointer.hh>
# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>
# include <pinocchio/spatial/inertia.hpp>
# include <pinocchio/multibody/fwd.hpp>

namespace hpp {
  namespace pinocchio {
    //DEPREC using fcl::Transform3f;

    /// Geometry associated to a Joint
    ///
    /// A body is a geometry container attached to a joint.
    /// The body contains objects (CollisionObject) that move with the joint and
    /// called <em>inner objects</em>.
    ///
    /// Collision and distance computation is performed against other objects
    /// that can be obstacles or objects attached to other joints. These object
    /// are called <em>outer objects</em> for the body.
    class HPP_PINOCCHIO_DLLAPI Body
    {
    public:

      typedef se3::JointIndex JointIndex;
      typedef se3::FrameIndex FrameIndex;

      /// \name Construction and copy and destruction
      /// @{
      /// Constructor
      Body (DevicePtr_t device, JointIndex joint);

      //DEPREC /// Copy constructor
      //DEPREC Body (const Body& body);
      //DEPREC /// Clone body and attach to given joint
      //DEPREC ///
      //DEPREC /// inner and outer object lists are filled with copies of the objects
      //DEPREC /// contained in the lists of this. (See CollisionObject::clone).
      //DEPREC BodyPtr_t clone (const JointPtr_t& joint) const;

      virtual ~Body () {}
      /// @}

      /// \name Name
      /// \{

      //DEPREC /// Set name
      //DEPREC void name (const std::string& name);
      /// Get name
      const std::string & name () const;
      /// \}

      /// Get joint holding the body
      JointPtr_t joint () const;

      //DEPREC /// Set joint holding the body
      //DEPREC void joint (JointPtr_t joint) {joint_ = joint;}

      /// \name Inner/outer objects
      /// \{

      //DEPREC /// Add an object to the body
      //DEPREC /// \param object object to add. Position of object is expressed in
      //DEPREC /// world frame.
      //DEPREC /// \param collision whether this object should be considered for
      //DEPREC ///        collision
      //DEPREC /// \param distance  whether this object should be considered for
      //DEPREC ///        distance computation
      //DEPREC /// \note If object is already in body, do nothing.
      //DEPREC virtual void addInnerObject (const CollisionObjectPtr_t& object,
      //DEPREC                              bool collision, bool distance);

      //DEPREC /// Remove an object to the body
      //DEPREC ///
      //DEPREC /// \param object object to remove
      //DEPREC /// \param collision whether this object should be removed from
      //DEPREC ///        list of collision objects
      //DEPREC /// \param distance  whether this object should be removed from
      //DEPREC ///        list of distance computation objects
      //DEPREC /// \note If object is not in body, do nothing
      //DEPREC virtual void removeInnerObject (const CollisionObjectPtr_t& object,
      //DEPREC 				      bool collision, bool distance);

      /// Access to inner objects
      /// \param type Collision or distance
//NOTYET      const ObjectVector_t& innerObjects (Request_t type) const;

      /// Get radius
      ///
      /// Radius is defined as an upper-bound to the distance of all points of
      /// the body to the origin of the joint that holds the body.
//NOTYET      value_type radius () const
      /// \}

      //DEPREC /// \name Outer objects
      //DEPREC /// \{

      //DEPREC /// Add an object as obstacle for this body
      //DEPREC /// \param object object to add. Position of object is expressed in
      //DEPREC /// world frame.
      //DEPREC /// \param collision whether this object should be considered for
      //DEPREC ///        collision
      //DEPREC /// \param distance  whether this object should be considered for
      //DEPREC ///        distance computation
      //DEPREC /// \note If object is already in body, do nothing.
      //DEPREC /// \warning Added objects by this method will be unknown from the
      //DEPREC ///         Device and will not be reset by it
      //DEPREC virtual void addOuterObject (const CollisionObjectPtr_t& object,
      //DEPREC 				   bool collision, bool distance);
      //DEPREC /// Remove an obstacle to the body
      //DEPREC ///
      //DEPREC /// \param object object to remove
      //DEPREC /// \param collision whether this object should be removed from
      //DEPREC ///        list of collision objects
      //DEPREC /// \param distance  whether this object should be removed from
      //DEPREC ///        list of distance computation objects
      //DEPREC /// \note If object is not in body, do nothing
      //DEPREC virtual void removeOuterObject (const CollisionObjectPtr_t& object,
      //DEPREC 				      bool collision, bool distance);

      /// Access to outer objects
      /// \param type Collision or distance
//NOTYET      const ObjectVector_t& outerObjects (Request_t type) const;
      /// \}

      /// \name Collision and distance computation
      /// @{

      /// Test for collision
      /// \return true if collision, false if no collision
//NOTYET      bool collisionTest () const;

      /// Compute distances between pairs of objects stored in bodies
//NOTYET      void computeDistances (DistanceResults_t& results,
//NOTYET			     DistanceResults_t::size_type& offset);

      /// @}
      /// \name Inertial information
      /// @{
      /// Get position of center of mass in joint local reference frame.
      vector3_t localCenterOfMass () const;
      /// Get Intertia matrix expressed in joint local reference frame.
      matrix3_t inertiaMatrix() const;
      /// Get mass.
      value_type mass() const;

      //DEPREC /// Set postion of center of mass in joint reference frame.
      //DEPREC inline  void localCenterOfMass (const fcl::Vec3f& localCenterOfMass);
      //DEPREC /// Set inertia matrix.
      //DEPREC inline  void inertiaMatrix(const matrix3_t& inertiaMatrix);
      //DEPREC /// Set mass.
      //DEPREC inline  void mass(value_type mass);


      ///  @}
    private:
      /// Assert that the members of the struct are valid (no null pointer, etc).
      void selfAssert() const;
      /// If frameIndex==-1 (after init), search in pinocchio frame list the proper index.
      void searchFrameIndex() const; 

      ModelConstPtr_t    model() const ;
      ModelPtr_t         model() ;
      const se3::Frame & frame() const ;
      se3::Frame &       frame() ;

//NOTYET      void updateRadius (const CollisionObjectPtr_t& object);
//NOTYET      ObjectVector_t collisionInnerObjects_;
//NOTYET      ObjectVector_t collisionOuterObjects_;
//NOTYET      ObjectVector_t distanceInnerObjects_;
//NOTYET      ObjectVector_t distanceOuterObjects_;
      DevicePtr_t devicePtr;
      JointIndex jointIndex;
      mutable FrameIndex frameIndex; // In pinocchio, bodies are stored as frames of type BODY.
      mutable bool       frameIndexSet;

      //DEPREC JointPtr_t joint_;
      //DEPREC std::string name_;
      //DEPREC /// Inertial information
      //DEPREC fcl::Vec3f localCom_;
      //DEPREC matrix3_t inertiaMatrix_;
      //DEPREC value_type mass_;
//NOTYET      value_type radius_;
    }; // class Body
  } // namespace pinocchio
} // namespace hpp
#endif // HPP_PINOCCHIO_BODY_HH
