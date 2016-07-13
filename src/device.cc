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

#include <Eigen/Core>
#include <hpp/pinocchio/fwd.hh>
//#include <hpp/pinocchio/distance-result.hh>
#include <hpp/pinocchio/extra-config-space.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/collisions.hpp>
#include <boost/foreach.hpp>

namespace hpp {
  namespace pinocchio {

    Device::
    Device(const std::string& name)
      : model_()
      , data_ ()
      , name_ (name)
      , jointVector_()
      , obstacles_()
      , weakPtr_()
    {}

    // static method
    DevicePtr_t Device::
    create (const std::string & name)
    {
      DevicePtr_t res = DevicePtr_t(new Device(name)); // init shared ptr
      res->weakPtr_ = res;
      res->jointVector_ = JointVector(res);
      res->obstacles_ = ObjectVector(res,0,CollisionObject::INNER);
      return res;
    }
    
    // static method
    DevicePtr_t Device::
    createCopy (const DevicePtr_t& device)
    {
      DevicePtr_t res = Device::create(device->name()); // init shared ptr
      res->model(device->model());  // Copy pointer to pinocchio model
      res->createData();    // Create a new data, dont copy the pointer.
      return res;
    }

    // static method
    DevicePtr_t Device::
    createCopyConst (const DeviceConstPtr_t& device)
    {
      DevicePtr_t res = Device::create(device->name()); // init shared ptr
      /* The copy of Pinocchio::Model is not implemented yet. */
      /* We need this feature to finish the implementation of this method. */
      assert( false && "TODO: createCopyConst is not implemented yet." );
      return res;
    }

    void Device::
    createData()
    {
      data_ = DataPtr_t( new se3::Data(*model_) );
      resizeState();
    }
    
    /* ---------------------------------------------------------------------- */
    /* --- JOINT ------------------------------------------------------------ */
    /* ---------------------------------------------------------------------- */

    JointPtr_t Device::
    rootJoint () const
    {
      return JointPtr_t( new Joint(weakPtr_.lock(),1) );
    }

    JointPtr_t Device::
    getJointAtConfigRank (const size_type& r) const
    {
      assert(model_);
      //BOOST_FOREACH( const se3::JointModel & j, // Skip "universe" joint
      //std::make_pair(model_->joints.begin()+1,model_->joints.end()) )
      BOOST_FOREACH( const se3::JointModel & j, model_->joints )
        {
          if( j.id()==0 ) continue; // Skip "universe" joint
          if( j.idx_q() == r ) return JointPtr_t( new Joint(weakPtr_.lock(),j.id()) );
        }
      assert(false && "The joint at config rank has not been found");
      return JointPtr_t();
    }

    JointPtr_t Device::
    getJointAtVelocityRank (const size_type& r) const
    {
      assert(model_);
      BOOST_FOREACH( const se3::JointModel & j,model_->joints )
        {
          if( j.id()==0 ) continue; // Skip "universe" joint
          if( j.idx_v() == r ) return JointPtr_t( new Joint(weakPtr_.lock(),j.id()) );
        }
      assert(false && "The joint at velocity rank has not been found");
      return JointPtr_t();
    }

    JointPtr_t Device::
    getJointByName (const std::string& name) const
    {
      assert(model_);
      if(! model_->existJointName(name))
	throw std::runtime_error ("Device " + name_ +
				  " does not have any joint named "
				  + name);
      se3::Index id = model_->getJointId(name);
      return JointPtr_t( new Joint(weakPtr_.lock(),id) );
    }

    JointPtr_t Device::
    getJointByBodyName (const std::string& name) const
    {
      assert(model_);
      assert(model_->existFrame(name));
      se3::Model::FrameIndex bodyId = model_->getFrameId(name);
      assert(model_->frames[bodyId].type == se3::BODY);
      se3::Model::JointIndex jointId = model_->frames[bodyId].parent;
      //assert(jointId>=0);
      assert((int)jointId<model_->njoint);
      return JointPtr_t( new Joint(weakPtr_.lock(),jointId) );
    }

    size_type Device::
    configSize () const
    {
      assert(model_);
      return model_->nq+extraConfigSpace_.dimension();
    }

    size_type Device::
    numberDof () const
    {
      assert(model_);
      return model_->nv;
    }

    /* ---------------------------------------------------------------------- */
    /* --- CONFIG ----------------------------------------------------------- */
    /* ---------------------------------------------------------------------- */

    /* Previous implementation of resizeState in hpp::model:: was setting the
     * new part of the configuration to neutral configuration. This is not
     * working but for empty extra-config. The former behavior is therefore not
     * propagated here. The configuration is resized without setting the new
     * memory.
     */
    void Device::
    resizeState()
    {
      currentConfiguration_.resize(configSize());
      currentVelocity_.resize(numberDof());
      currentAcceleration_.resize(numberDof());
    }

    bool Device::
    currentConfiguration (ConfigurationIn_t configuration)
    {
      if (configuration != currentConfiguration_)
        {
          upToDate_ = false;
          currentConfiguration_ = configuration;
          return true;
	}
      return false;
    }

    const value_type& Device::
    mass () const 
    { 
      assert(data_);
      return data_->mass[0];
    }
    
    vector3_t Device::
    positionCenterOfMass () const
    {
      assert(data_);
      return data_->com[0];
    }
    
    const ComJacobian_t& Device::
    jacobianCenterOfMass () const
    {
      assert(data_);
      return data_->Jcom;
    }

    void Device::
    computeForwardKinematics ()
    {
      if(upToDate_) return;

      assert(model_);
      assert(data_);
      // a IMPLIES b === (b || ~a)
      // velocity IMPLIES position
      assert( (computationFlag_|JOINT_POSITION) || (!(computationFlag_|VELOCITY)) );
      // acceleration IMPLIES velocity
      assert( (computationFlag_|VELOCITY) || (!(computationFlag_|ACCELERATION)) );
      // com IMPLIES position
      assert( (computationFlag_|JOINT_POSITION) || (!(computationFlag_|COM)) );
      // jacobian IMPLIES position
      assert( (computationFlag_|JOINT_POSITION) || (!(computationFlag_|JACOBIAN)) );

      if (computationFlag_ | ACCELERATION )
        se3::forwardKinematics(*model_,*data_,currentConfiguration_,
                               currentVelocity_,currentAcceleration_);
      else if (computationFlag_ | VELOCITY )
        se3::forwardKinematics(*model_,*data_,currentConfiguration_,
                               currentVelocity_);
      else if (computationFlag_ | JOINT_POSITION )
        se3::forwardKinematics(*model_,*data_,currentConfiguration_);

      if (computationFlag_|COM)
        {
          if (computationFlag_|JACOBIAN) 
            // TODO: Jcom should not recompute the kinematics (\sa pinocchio issue #219)
            se3::jacobianCenterOfMass(*model_,*data_,currentConfiguration_,true);
          else 
            se3::centerOfMass(*model_,*data_,currentConfiguration_,true,false);
        }

      if(computationFlag_|JACOBIAN)
        se3::computeJacobians(*model_,*data_,currentConfiguration_);
    }

    std::ostream& Device::
    print (std::ostream& os) const
    {
//NOTYET
      return os;
    }

    /* ---------------------------------------------------------------------- */
    /* --- COLLISIONS ------------------------------------------------------- */
    /* ---------------------------------------------------------------------- */

    bool Device::collisionTest (const bool stopAtFirstCollision)
    {
      /* Following hpp::model API, the forward kinematics (joint placement) is
       * supposed to have already been computed. */
      se3::updateGeometryPlacements(*model(),*data(),*geomModel(),*geomData());
      return se3::computeCollisions(*geomData(),stopAtFirstCollision);
    }

    void Device::computeDistances ()
    {
      /* Following hpp::model API, the forward kinematics (joint placement) is
       * supposed to have already been computed. */
      se3::updateGeometryPlacements(*model(),*data(),*geomModel(),*geomData());
      se3::computeDistances (*geomData());
    }

    const DistanceResults_t& Device::distanceResults () const
    {
      return geomData()->distance_results;
    }


  } // namespace pinocchio
} // namespace hpp
