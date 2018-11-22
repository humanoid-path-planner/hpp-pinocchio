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

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> // se3::details::Dispatch
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/fwd.hh>
//#include <hpp/pinocchio/distance-result.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/extra-config-space.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/pinocchio/liegroup-space.hh>

namespace hpp {
  namespace pinocchio {

    Device::
    Device(const std::string& name)
      : AbstractDevice ()
      , name_ (name)
      , weakPtr_()
      , datasLastFree_ (0)
    {
      invalidate();
      createData();
      createGeomData();

      numberDeviceData(1);
    }

    Device::Device(const Device& other)
      : AbstractDevice (other.model_, other.geomModel_)
      , d_ (other.d_)
      , name_ (other.name_)
      , grippers_ ()
      , extraConfigSpace_ (other.extraConfigSpace_)
      , weakPtr_()
      , datas_ ()
      , datasLastFree_ (0)
    {
      numberDeviceData(other.datas_.size());
    }

    Device::~Device ()
    {
      for (std::size_t i = 0; i < datas_.size(); ++i) delete datas_[i];
    }

    void Device::numberDeviceData (const size_type& s)
    {
      boost::mutex::scoped_lock lock (datasMutex_);
      if (datasLastFree_ > 0)
        throw std::logic_error ("Cannot set the number of device data when some"
            " of them are already in use.");
      size_type curSize = (size_type)datas_.size();
      // Delete if too many device data
      for (size_type i = 0; i < curSize; ++i) delete datas_[i];
      datas_.resize(s);
      // Initialize if too few device data
      for (size_type i = 0; i < s; ++i) datas_[i] = new DeviceData (d_);
    }

    size_type Device::numberDeviceData () const
    {
      return (size_type)datas_.size();
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
      d_.devicePtr_ = weakPtr;
      DevicePtr_t self (weakPtr_.lock());
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
      d_.data_ = DataPtr_t( new Data(model()) );
      // We assume that model is now complete and state can be resized.
      resizeState(); 
      d_.invalidate();
      numberDeviceData(datas_.size());
    }

    void Device::
    createGeomData()
    {
      d().geomData_ = GeomDataPtr_t( new GeomData(geomModel()) );
      se3::computeBodyRadius(model(),geomModel(),geomData());
      d().invalidate();
      numberDeviceData(datas_.size());
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

    size_type Device::nbJoints () const
    {
      return size_type(model().joints.size() - 1);
    }

    JointPtr_t Device::jointAt (const size_type& i) const
    {
      assert (i < nbJoints());
      return JointPtr_t(new Joint(weakPtr_.lock(),i+1));
    }

    JointPtr_t Device::
    getJointAtConfigRank (const size_type& r) const
    {
      //BOOST_FOREACH( const se3::JointModel & j, // Skip "universe" joint
      //std::make_pair(model_->joints.begin()+1,model_->joints.end()) )
      BOOST_FOREACH( const se3::JointModel & j, model().joints )
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
      BOOST_FOREACH( const se3::JointModel & j,model().joints )
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
      if(! model().existJointName(name))
	throw std::runtime_error ("Device " + name_ +
				  " does not have any joint named "
				  + name);
      JointIndex id = model().getJointId(name);
      return JointPtr_t( new Joint(weakPtr_.lock(),id) );
    }

    JointPtr_t Device::
    getJointByBodyName (const std::string& name) const
    {
      if (model().existFrame(name)) {
        se3::Model::FrameIndex bodyId = model().getFrameId(name);
        if (model().frames[bodyId].type == se3::BODY) {
          JointIndex jointId = model().frames[bodyId].parent;
          //assert(jointId>=0);
          assert((std::size_t)jointId<model().joints.size());
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
      const se3::FrameType type = (se3::FrameType) (se3::JOINT | se3::FIXED_JOINT);
      if(! model().existFrame(name, type))
	throw std::logic_error ("Device " + name_ +
				" does not have any frame named "
				+ name);
      FrameIndex id = model().getFrameId(name, type);
      return Frame(weakPtr_.lock(), id);
    }

    size_type Device::
    configSize () const
    {
      return model().nq + extraConfigSpace_.dimension();
    }

    size_type Device::
    numberDof () const
    {
      return model().nv + extraConfigSpace_.dimension();
    }

    /* ---------------------------------------------------------------------- */
    /* --- CONFIG ----------------------------------------------------------- */
    /* ---------------------------------------------------------------------- */

    void Device::
    resizeState()
    {
      d_.currentConfiguration_ = neutralConfiguration();
      d_.currentVelocity_      = vector_t::Zero(numberDof());
      d_.currentAcceleration_  = vector_t::Zero(numberDof());
      d_.jointJacobians_.resize (model().njoints);

      configSpace_ = LiegroupSpace::empty();
      const Model& m (model());
      for (JointIndex i = 1; i < m.joints.size(); ++i)
        *configSpace_ *= Joint(weakPtr_, i).configurationSpace();
      if (extraConfigSpace_.dimension() > 0)
        *configSpace_ *= LiegroupSpace::create (extraConfigSpace_.dimension());
    }

    LiegroupSpacePtr_t Device::
    RnxSOnConfigSpace () const
    {
      const Model& m (model());
      LiegroupSpacePtr_t space (LiegroupSpace::empty());
      for (JointIndex i = 1; i < m.joints.size(); ++i)
        *space *= Joint(weakPtr_, i).RnxSOnConfigurationSpace();
      if (extraConfigSpace_.dimension() > 0)
        *space *= LiegroupSpace::create (extraConfigSpace_.dimension());
      return space;
    }

    Configuration_t Device::
    neutralConfiguration () const
    {
      Configuration_t n (configSize());
      n.head(model().nq) = model().neutralConfiguration;
      n.tail(extraConfigSpace_.dimension()).setZero();
      return n;
    }

    std::ostream& Device::
    print (std::ostream& os) const
    {
      for (size_type i = 0; i < nbJoints(); ++i)
        jointAt(i)->display(os);
      return os;
    }

    /* ---------------------------------------------------------------------- */
    /* --- COLLISIONS ------------------------------------------------------- */
    /* ---------------------------------------------------------------------- */

    BodyPtr_t Device::obstacles () const
    {
      return BodyPtr_t( new Body(weakPtr_.lock(),0) );
    }

    size_type Device::nbObjects() const
    {
      return (size_type)geomModel().geometryObjects.size();
    }

    CollisionObjectPtr_t Device::objectAt (const size_type& i) const
    {
      assert (i < nbObjects());
      return CollisionObjectPtr_t (new CollisionObject(weakPtr_.lock(),i));
    }

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
        se3::JointModelComposite::JointDataDerived data = jmodel.createData();
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
      Configuration_t q (neutralConfiguration());
      fcl::AABB aabb;
      for (std::size_t k = 0; k < rootIdxs.size(); ++k)
      {
        JointIndex i = rootIdxs[k];
        value_type radius = maxRadius[k];

        fcl::AABB aabb_subtree;
        AABBStep::run(m.joints[i],
            AABBStep::ArgsType (m, q, true, aabb_subtree));

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
