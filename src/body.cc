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

#include <hpp/pinocchio/body.hh>

#include <boost/foreach.hpp>

#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/collision-object.hh>

namespace hpp {
  namespace pinocchio {

    Body::
    Body (DeviceWkPtr_t device, JointIndex joint) 
      : devicePtr(device),jointIndex(joint) ,frameIndexSet(false)
      , innerObjects_(device,joint,INNER)
      , outerObjects_(device,joint,OUTER)
    {
      selfAssert();
    }

    void Body::selfAssert() const 
    {
      DevicePtr_t device = devicePtr.lock();
      assert(device);
      assert(device->modelPtr());
      assert(device->model().joints.size()>std::size_t(jointIndex));
      if(frameIndexSet)
        assert(device->model().frames.size()>std::size_t(frameIndex));
    }

    const Model & Body::model() const { return devicePtr.lock()->model(); }
    Model &       Body::model()       { return devicePtr.lock()->model(); }
    se3::Frame &       Body::frame()
    {
      searchFrameIndex();
      return model().frames[frameIndex];
    }
    const se3::Frame & Body::frame() const
    {
      searchFrameIndex();
      return model().frames[frameIndex];
    }

    void Body::searchFrameIndex() const
    {
      if(frameIndexSet) return;
      frameIndex = 0;
      BOOST_FOREACH(const se3::Frame & frame,model().frames)
        {
          if( (se3::BODY == frame.type) && (frame.parent == jointIndex) )
            break;
          frameIndex++;
        }
      // If index not find, then it is set to size() -> normal behavior
      frameIndexSet = true;
    }

    const std::string & Body::name () const
    {
      selfAssert();
      return frame().name;
    }

    JointPtr_t Body::joint () const
    {
      selfAssert();
      return JointPtr_t( new Joint(devicePtr,jointIndex) );
    }


    const vector3_t& Body::localCenterOfMass () const
    {
      selfAssert();
      return model().inertias[jointIndex].lever();
    }
    matrix3_t Body::inertiaMatrix() const
    {
      selfAssert();
      return model().inertias[jointIndex].inertia();
    }
    value_type Body::mass() const
    {
      selfAssert();
      return model().inertias[jointIndex].mass();
    }

    size_type Body::nbInnerObjects () const
    {
      selfAssert();
      DevicePtr_t device = devicePtr.lock();
      return (size_type)device->geomData().innerObjects[jointIndex].size();
    }

    CollisionObjectPtr_t Body::innerObjectAt (const size_type& i) const
    {
      selfAssert();
      assert (0 <= i && i < nbInnerObjects());
      DevicePtr_t device = devicePtr.lock();
      return CollisionObjectPtr_t(new CollisionObject(device,
            device->geomData().innerObjects[jointIndex][i]));
    }

    value_type Body::radius () const
    {
      selfAssert();
      DevicePtr_t device = devicePtr.lock();
      assert(device->geomDataPtr());
      assert(std::size_t(device->geomData().radius.size())==model().joints.size());
      return device->geomData().radius[jointIndex]; 
    }

  } // namespace pinocchio
} // namespace hpp

