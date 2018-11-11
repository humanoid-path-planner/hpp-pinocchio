// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-pinocchio.
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
// hpp-pinocchio. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/pinocchio/device-data.hh>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/joint-collection.hh>

namespace hpp {
  namespace pinocchio {

    DeviceData::DeviceData ()
      : computationFlag_ (Computation_t(JOINT_POSITION | JACOBIAN))
    {
      invalidate();
    }

    DeviceData::DeviceData (const DeviceData& other)
      : data_     (new Data     (*other.data_    ))
      , geomData_ (new GeomData (*other.geomData_))
      , currentConfiguration_ (other.currentConfiguration_)
      , currentVelocity_      (other.currentVelocity_)
      , currentAcceleration_  (other.currentAcceleration_)
      , upToDate_        (other.upToDate_)
      , frameUpToDate_   (other.frameUpToDate_)
      , geomUpToDate_    (other.geomUpToDate_)
      , computationFlag_ (other.computationFlag_)
      , devicePtr_  ()
      , modelConf_      (other.modelConf_.size())
      , jointJacobians_ (other.jointJacobians_.size())
    {}

    void DeviceData::computeForwardKinematics (const ModelPtr_t& mptr)
    {
      if(upToDate_) return;
      const Model& model = *mptr;

      // a IMPLIES b === (b || ~a)
      // velocity IMPLIES position
      assert( (computationFlag_&JOINT_POSITION) || (!(computationFlag_&VELOCITY)) );
      // acceleration IMPLIES velocity
      assert( (computationFlag_&VELOCITY) || (!(computationFlag_&ACCELERATION)) );
      // com IMPLIES position
      assert( (computationFlag_&JOINT_POSITION) || (!(computationFlag_&COM)) );
      // jacobian IMPLIES position
      assert( (computationFlag_&JOINT_POSITION) || (!(computationFlag_&JACOBIAN)) );

      const size_type nq = model.nq;
      const size_type nv = model.nv;

      // TODO pinocchio does not allow to pass currentConfiguration_.head(nq) as
      // a reference. This line avoids dynamic memory allocation
      modelConf_ = currentConfiguration_.head(nq);

      if (computationFlag_ & ACCELERATION )
        ::pinocchio::forwardKinematics(model,*data_,modelConf_,
                               currentVelocity_.head(nv),currentAcceleration_.head(nv));
      else if (computationFlag_ & VELOCITY )
        ::pinocchio::forwardKinematics(model,*data_,modelConf_,
                               currentVelocity_.head(nv));
      else if (computationFlag_ & JOINT_POSITION )
        ::pinocchio::forwardKinematics(model,*data_,modelConf_);

      if (computationFlag_&COM)
        {
          if (computationFlag_|JACOBIAN) 
            // TODO: Jcom should not recompute the kinematics (\sa pinocchio issue #219)
            ::pinocchio::jacobianCenterOfMass(model,*data_,modelConf_,true);
          else 
            // 0 means Compose Com position, but not velocity and acceleration.
            ::pinocchio::centerOfMass(model,*data_,0,true);
        }

      if(computationFlag_&JACOBIAN)
        ::pinocchio::computeJointJacobians(model,*data_,modelConf_);

      upToDate_ = true;
    }

    void DeviceData::computeFramesForwardKinematics (const ModelPtr_t& mptr)
    {
      if(frameUpToDate_) return;
      computeForwardKinematics(mptr);

      ::pinocchio::updateFramePlacements (*mptr,*data_);

      frameUpToDate_ = true;
    }

    void DeviceData::updateGeometryPlacements (const ModelPtr_t& m, const GeomModelPtr_t& gm)
    {
      if (!geomUpToDate_) {
        ::pinocchio::updateGeometryPlacements(*m,*data_,*gm,*geomData_);
        geomUpToDate_ = true;
      }
    }
  } // namespace pinocchio
} // namespace hpp
