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

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>

namespace hpp {
  namespace pinocchio {

    CollisionObject::
    CollisionObject( DevicePtr_t device, 
                     const GeomIndex geomInModel )
      : devicePtr(device)
      , geomModel_(devicePtr->geomModelPtr())
      , jointIndex_(0)
      , geomInModelIndex(geomInModel)
    {
      jointIndex_ = pinocchio().parentJoint;
      selfAssert();
    }

    CollisionObject::
    CollisionObject( GeomModelPtr_t geomModel,
                     GeomDataPtr_t  geomData,
                     const GeomIndex geomInModel )
      : devicePtr()
      , geomModel_(geomModel)
      , geomData_(geomData)
      , jointIndex_(0)
      , geomInModelIndex(geomInModel)
    {
      jointIndex_ = pinocchio().parentJoint;
      selfAssert();
    }
    
    const std::string& CollisionObject::name () const { return pinocchio().name; }

    // This function should rather return a shared_ptr<const>
    const se3::GeometryObject & CollisionObject::pinocchio () const
    { return geomModel_->geometryObjects[geomInModelIndex]; }
    se3::GeometryObject & CollisionObject::pinocchio ()
    { return geomModel_->geometryObjects[geomInModelIndex]; }

    FclCollisionObjectPtr_t CollisionObject::fcl (GeomData& geomData) const
    {
      return & geomData.collisionObjects[geomInModelIndex];
    }

    FclConstCollisionObjectPtr_t CollisionObject::fcl (const GeomData& geomData) const
    {
      return & geomData.collisionObjects[geomInModelIndex];
    }

    FclConstCollisionObjectPtr_t CollisionObject::fcl () const { return fcl(geomData()); }
    FclCollisionObjectPtr_t      CollisionObject::fcl ()       { return fcl(geomData()); }

    FclCollisionObjectPtr_t CollisionObject::fcl (DeviceData& d) const
    {
      return & geomData(d).collisionObjects[geomInModelIndex];
    }

    JointPtr_t      CollisionObject::joint ()
    {
      if (!devicePtr) return JointPtr_t();
      return Joint::create (devicePtr,jointIndex_);
    }

    JointConstPtr_t CollisionObject::joint () const
    {
      if (!devicePtr) return JointConstPtr_t();
      return Joint::create (devicePtr,jointIndex_);
    }

    const Transform3f& CollisionObject::
    positionInJointFrame () const { return pinocchio().placement; }

    const fcl::Transform3f& CollisionObject::getFclTransform () const
    { return geomData().collisionObjects[geomInModelIndex].getTransform(); }
    const Transform3f& CollisionObject::getTransform () const
    { return geomData().oMg[geomInModelIndex];  }
    const Transform3f& CollisionObject::getTransform (DeviceData& d) const
    { return geomData(d).oMg[geomInModelIndex];  }

    void CollisionObject::move (const Transform3f& position)
    { 
      // move does not work but for object attached to the universe (joint 0)
      assert( jointIndex_==0 ); 
      geomData().oMg[geomInModelIndex] = position;
      geomData().collisionObjects[geomInModelIndex]
        .setTransform(toFclTransform3f(position));
      pinocchio().placement = position;
    }

    void CollisionObject::selfAssert() const
    { 
      assert(geomModel_);
      assert(devicePtr || geomData_);
      assert(!devicePtr || devicePtr->model().joints.size()>std::size_t(jointIndex_));
      assert(geomModel_->geometryObjects.size()>geomInModelIndex);
    }

    GeomData& CollisionObject::geomData () const
    {
      if (devicePtr) // Object of the robot.
        return geomData (devicePtr->d());
      else           // Object of the environment.
        return *geomData_;
    }

    GeomData& CollisionObject::geomData (DeviceData& d) const
    {
      if (geomData_) return *geomData_;
      else return *d.geomData_;
    }
  } // namespace pinocchio
} // namespace hpp
