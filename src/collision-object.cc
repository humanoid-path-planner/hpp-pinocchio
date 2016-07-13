//
// Copyright (c) 2016 CNRS
// Author: NMansard
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


#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/device.hh>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace hpp {
  namespace pinocchio {

    CollisionObject::
    CollisionObject( DevicePtr_t device, 
                     const JointIndex joint,
                     const GeomIndex geom,
                     const InOutType inout ) 
      : devicePtr(device),jointIndex(joint)
      , geomInJointIndex(geom),geomInModelIndex(0)
      , inOutType(inout)
    {
      selfAssert(); 
      geomInModelIndex = objectVec().at(jointIndex)[geomInJointIndex];
      selfAssert(); 
    }

    CollisionObject::ObjectVec_t & 
    CollisionObject::objectVec()
    {
      if(inOutType==INNER) return devicePtr->geomModel()->innerObjects;
      else                 return devicePtr->geomModel()->outerObjects;
    }
    const CollisionObject::ObjectVec_t & 
    CollisionObject::objectVec() const
    {
      if(inOutType==INNER) return devicePtr->geomModel()->innerObjects;
      else                 return devicePtr->geomModel()->outerObjects;
    }
    
    const std::string& CollisionObject::name () const { return pinocchio().name; }

    const se3::GeometryObject & CollisionObject::pinocchio () const
    { return devicePtr->geomModel()->geometry_objects[geomInModelIndex]; }
    se3::GeometryObject & CollisionObject::pinocchio ()
    { return devicePtr->geomModel()->geometry_objects[geomInModelIndex]; }

    fclCollisionObjectPtr_t CollisionObject::fcl ()
    { return & pinocchio().collision_object; }
    fclConstCollisionObjectPtr_t CollisionObject::fcl () const 
    { return & pinocchio().collision_object; }

    JointPtr_t CollisionObject::joint () { return JointPtr_t(new Joint(devicePtr,jointIndex)); }

    const Transform3f& CollisionObject::
    positionInJointFrame () const { return pinocchio().placement; }

    const fcl::Transform3f& CollisionObject::getFclTransform () const
    { return devicePtr->geomData()->oMg_fcl_geometries[geomInModelIndex]; }
    const Transform3f& CollisionObject::getTransform () const
    { return devicePtr->geomData()->oMg_geometries[geomInModelIndex];  }

    void CollisionObject::move (const Transform3f& position)
    { 
      // move does not work but for object attached to the universe (joint 0)
      assert( jointIndex==0 ); 
      devicePtr->geomData()->oMg_geometries[geomInModelIndex] = position;
      devicePtr->geomData()->oMg_fcl_geometries[geomInModelIndex]
        = toFclTransform3f(position);
      pinocchio().placement = position;
    }

    void CollisionObject::selfAssert() const
    { 
      assert(devicePtr);
      assert(devicePtr->model()->njoint>int(jointIndex));
      assert(objectVec().at(jointIndex).size()>geomInJointIndex);
      assert(devicePtr->geomModel()->geometry_objects.size()>geomInModelIndex);
    }

    /* --- ITERATOR --------------------------------------------------------- */
    CollisionObjectPtr_t ObjectVector::at(const size_type i)
    {
      return CollisionObjectPtr_t(new CollisionObject(devicePtr,jointIndex,i,inOutType));
    }

    CollisionObjectConstPtr_t ObjectVector::at(const size_type i) const
    {
      return CollisionObjectConstPtr_t(new CollisionObject(devicePtr,jointIndex,i,inOutType));
    }

    size_type ObjectVector::size() const
    {
      if( inOutType==CollisionObject::INNER )
        return size_type(devicePtr->geomModel()->innerObjects.size());
      else
        return size_type(devicePtr->geomModel()->outerObjects.size());
    }
    
    void ObjectVector::selfAssert(size_type i) const
    {
      assert(devicePtr);
      assert(int(jointIndex)<devicePtr->model()->njoint);
      assert(i<size());
    }

  } // namespace pinocchio
} // namespace hpp
