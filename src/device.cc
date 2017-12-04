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

#include <hpp/fcl/BV/AABB.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> // se3::details::Dispatch

#include <hpp/pinocchio/fwd.hh>
//#include <hpp/pinocchio/distance-result.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/extra-config-space.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup.hh>

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
      , frameUpToDate_ (false)
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
      invalidate();
    }

    void Device::
    createGeomData()
    {
      geomData_ = GeomDataPtr_t( new GeomData(*geomModel_) );
      se3::computeBodyRadius(*model_,*geomModel_,*geomData_);
      invalidate();
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

      // TODO pinocchio does not allow to pass currentConfiguration_.head(nq) as
      // a reference. This line avoids dynamic memory allocation
      robotConf_ = currentConfiguration_.head(nq);

      if (computationFlag_ & ACCELERATION )
        se3::forwardKinematics(*model_,*data_,robotConf_,
                               currentVelocity_.head(nv),currentAcceleration_.head(nv));
      else if (computationFlag_ & VELOCITY )
        se3::forwardKinematics(*model_,*data_,robotConf_,
                               currentVelocity_.head(nv));
      else if (computationFlag_ & JOINT_POSITION )
        se3::forwardKinematics(*model_,*data_,robotConf_);

      if (computationFlag_&COM)
        {
          if (computationFlag_|JACOBIAN) 
            // TODO: Jcom should not recompute the kinematics (\sa pinocchio issue #219)
            se3::jacobianCenterOfMass(*model_,*data_,robotConf_,true);
          else 
            // Compose Com position, but not velocity and acceleration.
            se3::centerOfMass<true, false, false>(*model_,*data_,true);
        }

      if(computationFlag_&JACOBIAN)
        se3::computeJacobians(*model_,*data_,robotConf_);

      upToDate_ = true;
    }

    void Device::
    computeFramesForwardKinematics ()
    {
      if(frameUpToDate_) return;
      computeForwardKinematics();

      se3::framesForwardKinematics (model(),data());

      frameUpToDate_ = true;
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

    /* ---------------------------------------------------------------------- */
    /* --- Bounding box ----------------------------------------------------- */
    /* ---------------------------------------------------------------------- */

    struct AABBStep : public se3::fusion::JointModelVisitor<AABBStep>
    {
      typedef boost::fusion::vector<const Model&,
                                    Configuration_t,
                                    bool,
                                    fcl::AABB&> ArgsType;

      JOINT_MODEL_VISITOR_INIT(AABBStep);

      template<typename JointModel>
      static void algo(const se3::JointModelBase<JointModel> & jmodel,
                       const Model& model,
                       Configuration_t q,
                       bool initializeAABB,
                       fcl::AABB& aabb)
      {
        typedef typename LieGroupTpl::operation<JointModel>::type LG_t;
        /*
        if (LG_t::NT == 0) {
          aabb.min_.setZero();
          aabb.max_.setZero();
          return;
        }
        */
        typename JointModel::JointDataDerived data (jmodel.createData());
        // Set configuration to lower bound.
        jmodel.jointConfigSelector(q).template head<LG_t::NT>() =
          jmodel.jointConfigSelector(model.lowerPositionLimit).template head<LG_t::NT>();
        jmodel.calc(data, q);
        vector3_t min = data.M.translation();
        // Set configuration to upper bound.
        jmodel.jointConfigSelector(q).template head<LG_t::NT>() =
          jmodel.jointConfigSelector(model.upperPositionLimit).template head<LG_t::NT>();
        jmodel.calc(data, q);
        vector3_t max = data.M.translation();

        // This should not be required as it should be done in AABB::operator+=(Vec3f)
        // for(int i = 0; i < 3; ++i) {
          // if (min[i] > max[i]) std::swap(min[i], max[i]);
        // }
        // aabb.min_ = min;
        // aabb.max_ = max;
        if (initializeAABB) aabb = fcl::AABB(min);
        else                aabb += min;
        aabb += max;
      }

    };

    template <>
    void AABBStep::algo<se3::JointModelComposite>(
        const se3::JointModelBase<se3::JointModelComposite> & jmodel,
        const Model& model,
        Configuration_t q,
        bool initializeAABB,
        fcl::AABB& aabb)
    {
      // TODO this should for but I did not test it.
      hppDout(warning, "Computing AABB of JointModelComposite should work but has never been tested");
      if (initializeAABB)  {
        typename se3::JointModelComposite::JointDataDerived data = jmodel.createData();
        jmodel.calc(data, q);
        aabb = fcl::AABB(data.M.translation());
      }
      se3::details::Dispatch<AABBStep>::run(jmodel, AABBStep::ArgsType(model, q, false, aabb));
    }

    fcl::AABB Device::computeAABB() const
    {
      // TODO check that user has called

      const Model& m (model());

      // Compute maximal distance to parent joint.
      std::vector<value_type> maxDistToParent (m.joints.size(), 0);
      for (JointIndex i = 1; i < m.joints.size(); ++i)
      {
        Joint joint (weakPtr_.lock(), i);
        joint.computeMaximalDistanceToParent();
        maxDistToParent[i] = joint.maximalDistanceToParent();
      }

      // Compute maximal distance to root joint.
      std::vector<value_type> maxDistToRoot (m.joints.size(), 0);
      std::vector<value_type> distances (m.joints.size(), 0);

      std::vector<JointIndex> rootIdxs;
      std::vector<value_type> maxRadius;
      for (JointIndex i = 1; i < m.joints.size(); ++i)
      {
        if (m.parents[i] == 0) // Moving child of universe
        {
          rootIdxs.push_back(i);
          maxRadius.push_back(0);
          // This is the root of a subtree.
          // maxDistToRoot[i] = 0; // Already zero by initialization
        } else {
          maxDistToRoot[i] = maxDistToRoot[m.parents[i]] + maxDistToParent[i];
        }

        Body body (weakPtr_.lock(), i);
        distances[i] = body.radius() + maxDistToRoot[i];

        maxRadius.back() = std::max(maxRadius.back(), distances[i]);
      }

      // Compute AABB
      fcl::AABB aabb;
      for (std::size_t k = 0; k < rootIdxs.size(); ++k)
      {
        JointIndex i = rootIdxs[k];
        value_type radius = maxRadius[k];

        fcl::AABB aabb_subtree;
        AABBStep::run(m.joints[i],
            AABBStep::ArgsType (m, currentConfiguration_, true, aabb_subtree));

        // Move AABB
        fcl::rotate   (aabb_subtree, m.jointPlacements[i].rotation   ());
        fcl::translate(aabb_subtree, m.jointPlacements[i].translation());

        // Add radius
        aabb_subtree.min_.array() -= radius;
        aabb_subtree.max_.array() += radius;
        
        // Merge back into previous one.
        if (k == 0) aabb  = aabb_subtree;
        else        aabb += aabb_subtree;
      }
      return aabb;
    }
  } // namespace pinocchio
} // namespace hpp
