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

#include <hpp/pinocchio/device.hh>

#include <boost/foreach.hpp>
#include <Eigen/Core>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>

#include <hpp/pinocchio/fwd.hh>
//#include <hpp/pinocchio/distance-result.hh>
#include <hpp/pinocchio/extra-config-space.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint.hh>

namespace hpp {
  namespace pinocchio {

    Device::
    Device(const std::string& name)
      : model_(new Model())
      , data_ ()
      , geomModel_(new GeomModel())
      , geomData_ ()
      , name_ (name)
      , jointVector_()
      , computationFlag_ (Computation_t(JOINT_POSITION | JACOBIAN))
      , obstacles_()
      , objectVector_ ()
      , weakPtr_()
    {
      invalidate();
      createData();
      createGeomData();
    }

    Device::Device(const Device& other)
      : model_(other.model_)
      , data_ (new Data (other.data()))
      , geomModel_(other.geomModel_)
      , geomData_ (new GeomData (other.geomData()))
      , name_ (other.name_)
      , jointVector_()
      , currentConfiguration_ (other.currentConfiguration_)
      , currentVelocity_ (other.currentVelocity_)
      , currentAcceleration_ (other.currentAcceleration_)
      , upToDate_ (false)
      , geomUpToDate_ (false)
      , computationFlag_ (other.computationFlag_)
      , obstacles_()
      , objectVector_ ()
      , grippers_ ()
      , extraConfigSpace_ (other.extraConfigSpace_)
      , weakPtr_()
    {
    }

    // static method
    DevicePtr_t Device::
    create (const std::string & name)
    {
      DevicePtr_t res = DevicePtr_t(new Device(name)); // init shared ptr
      res->init (res);
      return res;
    }
    
    // static method
    DevicePtr_t Device::
    createCopy (const DevicePtr_t& other)
    {
      DevicePtr_t res = DevicePtr_t(new Device(*other)); // init shared ptr
      res->initCopy (res, *other);
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

    void Device::init(const DeviceWkPtr_t& weakPtr)
    {
      weakPtr_ = weakPtr;
      DevicePtr_t self (weakPtr_.lock());
      jointVector_ = JointVector(self);
      obstacles_ = ObjectVector(self,0,INNER);
      objectVector_ = DeviceObjectVector(self);
    }

    void Device::initCopy(const DeviceWkPtr_t& weakPtr, const Device& other)
    {
      init(weakPtr);
      grippers_.resize (other.grippers_.size());
      for (std::size_t i = 0; i < grippers_.size(); ++i)
        grippers_[i] = Gripper::createCopy(other.grippers_[i], weakPtr);
    }

    void Device::
    createData()
    {
      data_ = DataPtr_t( new Data(*model_) );
      // We assume that model is now complete and state can be resized.
      resizeState(); 
    }

    void Device::
    createGeomData()
    {
      geomData_ = GeomDataPtr_t( new GeomData(*geomModel_) );
      se3::computeBodyRadius(*model_,*geomModel_,*geomData_);
    }
    
    /* ---------------------------------------------------------------------- */
    /* --- JOINT ------------------------------------------------------------ */
    /* ---------------------------------------------------------------------- */

    JointPtr_t Device::rootJoint () const
    {
      return JointPtr_t( new Joint(weakPtr_.lock(),1) );
    }

    Frame Device::rootFrame () const
    {
      const se3::FrameType type = (se3::FrameType)(se3::JOINT | se3::FIXED_JOINT);
      return Frame(weakPtr_.lock(), model().getFrameId("root_joint", type));
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
          const size_type iq = r - j.idx_q();
          if( 0 <= iq && iq < j.nq() ) return JointPtr_t( new Joint(weakPtr_.lock(),j.id()) );
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
          const size_type iv = r - j.idx_v();
          if( 0 <= iv && iv < j.nv() ) return JointPtr_t( new Joint(weakPtr_.lock(),j.id()) );
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
      JointIndex id = model_->getJointId(name);
      return JointPtr_t( new Joint(weakPtr_.lock(),id) );
    }

    JointPtr_t Device::
    getJointByBodyName (const std::string& name) const
    {
      assert(model_);
      if (model_->existFrame(name)) {
        se3::Model::FrameIndex bodyId = model_->getFrameId(name);
        if (model_->frames[bodyId].type == se3::BODY) {
          JointIndex jointId = model_->frames[bodyId].parent;
          //assert(jointId>=0);
          assert((std::size_t)jointId<model_->joints.size());
          return JointPtr_t( new Joint(weakPtr_.lock(),jointId) );
        }
      }
      throw std::runtime_error ("Device " + name_ +
                                " has no joint with body of name "
                                + name);
    }

    Frame Device::
    getFrameByName (const std::string& name) const
    {
      assert(model_);
      const se3::FrameType type = (se3::FrameType)(se3::JOINT | se3::FIXED_JOINT);
      if(! model_->existFrame(name, type))
	throw std::logic_error ("Device " + name_ +
				" does not have any frame named "
				+ name);
      FrameIndex id = model_->getFrameId(name, type);
      return Frame(weakPtr_.lock(), id);
    }

    size_type Device::
    configSize () const
    {
      assert(model_);
      return model_->nq + extraConfigSpace_.dimension();
    }

    size_type Device::
    numberDof () const
    {
      assert(model_);
      return model_->nv + extraConfigSpace_.dimension();
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
      // FIXME we should not use neutralConfiguration here.
      currentConfiguration_ = neutralConfiguration();
      // currentConfiguration_.resize(configSize());
      currentVelocity_.resize(numberDof());
      currentAcceleration_.resize(numberDof());
    }

    bool Device::
    currentConfiguration (ConfigurationIn_t configuration)
    {
      if (configuration != currentConfiguration_)
        {
          invalidate();
          currentConfiguration_ = configuration;
          return true;
	}
      return false;
    }

    Configuration_t Device::
    neutralConfiguration () const
    {
      Configuration_t n (configSize());
      n.head(model_->nq) = model().neutralConfiguration;
      n.tail(extraConfigSpace_.dimension()).setZero();
      return n;
    }

    const value_type& Device::
    mass () const 
    { 
      assert(data_);
      return data_->mass[0];
    }
    
    const vector3_t& Device::
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
      assert( (computationFlag_&JOINT_POSITION) || (!(computationFlag_&VELOCITY)) );
      // acceleration IMPLIES velocity
      assert( (computationFlag_&VELOCITY) || (!(computationFlag_&ACCELERATION)) );
      // com IMPLIES position
      assert( (computationFlag_&JOINT_POSITION) || (!(computationFlag_&COM)) );
      // jacobian IMPLIES position
      assert( (computationFlag_&JOINT_POSITION) || (!(computationFlag_&JACOBIAN)) );

      const size_type nq = model().nq;
      const size_type nv = model().nv;

      if (computationFlag_ & ACCELERATION )
        se3::forwardKinematics(*model_,*data_,currentConfiguration_.head(nq),
                               currentVelocity_.head(nv),currentAcceleration_.head(nv));
      else if (computationFlag_ & VELOCITY )
        se3::forwardKinematics(*model_,*data_,currentConfiguration_.head(nq),
                               currentVelocity_.head(nv));
      else if (computationFlag_ & JOINT_POSITION )
        se3::forwardKinematics(*model_,*data_,currentConfiguration_.head(nq));

      if (computationFlag_&COM)
        {
          if (computationFlag_|JACOBIAN) 
            // TODO: Jcom should not recompute the kinematics (\sa pinocchio issue #219)
            se3::jacobianCenterOfMass(*model_,*data_,currentConfiguration_.head(nq),true);
          else 
            // Compose Com position, but not velocity and acceleration.
            se3::centerOfMass<true, false, false>(*model_,*data_,true);
        }

      if(computationFlag_&JACOBIAN)
        se3::computeJacobians(*model_,*data_,currentConfiguration_.head(nq));
    }

    void Device::
    updateGeometryPlacements ()
    {
      if (!geomUpToDate_) {
        se3::updateGeometryPlacements(model(),data(),geomModel(),geomData());
        geomUpToDate_ = true;
      }
    }

    std::ostream& Device::
    print (std::ostream& os) const
    {
      for (JointVector::const_iterator it = jointVector_.begin (); it != jointVector_.end (); ++it) 
        (*it)->display(os);
      return os;
    }

    /* ---------------------------------------------------------------------- */
    /* --- COLLISIONS ------------------------------------------------------- */
    /* ---------------------------------------------------------------------- */

    bool Device::collisionTest (const bool stopAtFirstCollision)
    {
      /* Following hpp::model API, the forward kinematics (joint placement) is
       * supposed to have already been computed. */
      updateGeometryPlacements();
      return se3::computeCollisions(geomModel(), geomData(),stopAtFirstCollision);
    }

    void Device::computeDistances ()
    {
      /* Following hpp::model API, the forward kinematics (joint placement) is
       * supposed to have already been computed. */
      updateGeometryPlacements();
      se3::computeDistances (geomModel(), geomData());
    }

    const DistanceResults_t& Device::distanceResults () const
    {
      return geomData().distanceResults;
    }

  } // namespace pinocchio
} // namespace hpp
