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

namespace hpp {
  namespace pinocchio {

    Body::
    Body (DevicePtr_t device, JointIndex joint) 
      : devicePtr(device),jointIndex(joint) ,frameIndexSet(false)
      , innerObjects_(device,joint,INNER)
      , outerObjects_(device,joint,OUTER)
    {
      selfAssert();
    }

    void Body::selfAssert() const 
    {
      assert(devicePtr);
      assert(devicePtr->modelPtr());
      assert(devicePtr->model().njoint>int(jointIndex));
      if(frameIndexSet)
        assert(devicePtr->model().nFrames>int(frameIndex));
    }

    const Model & Body::model() const { return devicePtr->model(); }
    Model &       Body::model()       { return devicePtr->model(); }
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
      BOOST_FOREACH(const se3::Frame & frame,devicePtr->model().frames)
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

    value_type Body::radius () const
    {
      selfAssert();
      assert(devicePtr->geomDataPtr());
      assert(int(devicePtr->geomData().radius.size())==model().njoint);
      return devicePtr->geomData().radius[jointIndex]; 
    }

  } // namespace pinocchio
} // namespace hpp

