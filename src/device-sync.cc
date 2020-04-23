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

#include <hpp/pinocchio/device-sync.hh>

#include <boost/thread/locks.hpp>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>

namespace hpp {
  namespace pinocchio {
    bool AbstractDevice::currentConfiguration (ConfigurationIn_t configuration)
    {
      DeviceData& data = d();
      assert (configuration.size() == data.currentConfiguration_.size());
      if (configuration != data.currentConfiguration_) {
        data.invalidate();
        data.currentConfiguration_ = configuration;
        return true;
      }
      return false;
    }

    bool AbstractDevice::currentVelocity (vectorIn_t v)
    {
      DeviceData& data = d();
      if (v != data.currentVelocity_) {
        data.invalidate();
        data.currentVelocity_ = v;
        return true;
      }
      return false;
    }

    bool AbstractDevice::currentAcceleration (vectorIn_t a)
    {
      DeviceData& data = d();
      if (a != data.currentAcceleration_) {
        data.invalidate();
        data.currentAcceleration_ = a;
        return true;
      }
      return false;
    }

    const value_type& AbstractDevice::mass () const 
    { 
      return data().mass[0];
    }
    
    const vector3_t& AbstractDevice::positionCenterOfMass () const
    {
      return data().com[0];
    }
    
    const ComJacobian_t& AbstractDevice::jacobianCenterOfMass () const
    {
      return data().Jcom;
    }

    void AbstractDevice::controlComputation (const Computation_t& flag)
    {
      if (d().computationFlag_ != flag) {
        d().computationFlag_ = flag;
        d().invalidate();
      }
    }

    AbstractDevice::AbstractDevice ()
      : model_    (new Model())
      , geomModel_(new GeomModel())
    {}

    AbstractDevice::AbstractDevice (const ModelPtr_t& m, const GeomModelPtr_t& gm)
      : model_    (m)
      , geomModel_(gm)
    {}

    DeviceSync::DeviceSync (const DevicePtr_t& d, bool acquireLock)
      : AbstractDevice (d->modelPtr(), d->geomModelPtr())
      , device_ (d)
      , d_ (NULL)
    {
      if (acquireLock) lock();
    }

    DeviceSync::~DeviceSync ()
    {
      if (isLocked())
        unlock();
    }

    void DeviceSync::lock ()
    {
      if (!isLocked()) {
        d_ = device_->datas_.acquire();
      } else {
        hppDout (warning, "Cannot lock a locked DeviceSync. You may a concurrency error.");
      }
    }

    void DeviceSync::unlock ()
    {
      if (isLocked()) {
        device_->datas_.release(d_);
        d_ = NULL;
      } else {
        hppDout (warning, "Cannot unlock an unlocked DeviceSync. You may a concurrency error.");
      }
    }
  } // namespace pinocchio
} // namespace hpp
