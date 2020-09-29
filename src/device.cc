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
#include <boost/serialization/export.hpp>

#include <Eigen/Core>

#include <hpp/util/serialization.hh>

#include <hpp/fcl/BV/AABB.h>
#include <hpp/fcl/BVH/BVH_model.h>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> // ::pinocchio::details::Dispatch
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/serialization/model.hpp>

#include <hpp/pinocchio/fwd.hh>
#include <hpp/pinocchio/joint-collection.hh>
//#include <hpp/pinocchio/distance-result.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/extra-config-space.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <hpp/pinocchio/serialization.hh>

namespace hpp {
  namespace pinocchio {
    const ::pinocchio::FrameType all_joint_type =
      (::pinocchio::FrameType) (::pinocchio::JOINT | ::pinocchio::FIXED_JOINT);

    Device::
    Device(const std::string& name)
      : AbstractDevice ()
      , name_ (name)
      , weakPtr_()
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
    {
      numberDeviceData(other.numberDeviceData());
    }

    Device::~Device ()
    {
      datas_.clear();
    }

    void Device::numberDeviceData (const size_type& s)
    {
      // Delete current device datas
      datas_.clear();
      // Create new device datas
      std::vector<DeviceData*> datas ((std::size_t)s);
      for (std::size_t i = 0; i < (std::size_t)s; ++i) datas[i] = new DeviceData (d_);
      datas_.push_back (datas.begin(), datas.end());
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
    }

    void Device::
    createGeomData()
    {
      d().geomData_ = GeomDataPtr_t( new GeomData(geomModel()) );
      ::pinocchio::computeBodyRadius(model(),geomModel(),geomData());
      d().invalidate();
      numberDeviceData(numberDeviceData());
    }

    void Device::controlComputation (const Computation_t& flag)
    {
      AbstractDevice::controlComputation (flag);
      // TODO this should not be done in controlComputation
      // It should be done in another function (like controlComputations)
      // as it might be a desired behaviour to have different computation options
      // in different DeviceData.
      numberDeviceData(numberDeviceData());
    }
    
    /* ---------------------------------------------------------------------- */
    /* --- JOINT ------------------------------------------------------------ */
    /* ---------------------------------------------------------------------- */

    JointPtr_t Device::rootJoint () const
    {
      return Joint::create(weakPtr_.lock(),1);
    }

    Frame Device::rootFrame () const
    {
      return Frame(weakPtr_.lock(), model().getFrameId("root_joint", all_joint_type));
    }

    size_type Device::nbJoints () const
    {
      return size_type(model().joints.size() - 1);
    }

    JointPtr_t Device::jointAt (const size_type& i) const
    {
      assert (i < nbJoints());
      return Joint::create(weakPtr_.lock(),JointIndex(i+1));
    }

    JointPtr_t Device::
    getJointAtConfigRank (const size_type& r) const
    {
      //BOOST_FOREACH( const JointModel & j, // Skip "universe" joint
      //std::make_pair(model_->joints.begin()+1,model_->joints.end()) )
      BOOST_FOREACH( const JointModel & j, model().joints )
        {
          if( j.id()==0 ) continue; // Skip "universe" joint
          const size_type iq = r - j.idx_q();
          if( 0 <= iq && iq < j.nq() ) return Joint::create(weakPtr_.lock(),j.id());
        }
      assert(false && "The joint at config rank has not been found");
      return JointPtr_t();
    }

    JointPtr_t Device::
    getJointAtVelocityRank (const size_type& r) const
    {
      BOOST_FOREACH( const JointModel & j,model().joints )
        {
          if( j.id()==0 ) continue; // Skip "universe" joint
          const size_type iv = r - j.idx_v();
          if( 0 <= iv && iv < j.nv() ) return Joint::create(weakPtr_.lock(),j.id());
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
      return Joint::create(weakPtr_.lock(),id);
    }

    JointPtr_t Device::
    getJointByBodyName (const std::string& name) const
    {
      if (model().existFrame(name)) {
        FrameIndex bodyId = model().getFrameId(name);
        if (model().frames[bodyId].type == ::pinocchio::BODY) {
          JointIndex jointId = model().frames[bodyId].parent;
          //assert(jointId>=0);
          assert((std::size_t)jointId<model().joints.size());
          return Joint::create(weakPtr_.lock(),jointId);
        }
      }
      throw std::runtime_error ("Device " + name_ +
                                " has no joint with body of name "
                                + name);
    }

    Frame Device::
    getFrameByName (const std::string& name) const
    {
      if(! model().existFrame(name))
	throw std::logic_error ("Device " + name_ +
				" does not have any frame named "
				+ name);
      FrameIndex id = model().getFrameId(name);
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
      d_.invalidate();
      d_.currentConfiguration_ = neutralConfiguration();
      d_.currentVelocity_      = vector_t::Zero(numberDof());
      d_.currentAcceleration_  = vector_t::Zero(numberDof());
      d_.jointJacobians_.resize ((std::size_t)model().njoints);

      configSpace_ = LiegroupSpace::empty();
      configSpaceRnxSOn_ = LiegroupSpace::empty();
      const Model& m (model());
      for (JointIndex i = 1; i < m.joints.size(); ++i) {
        *configSpace_       *= Joint(weakPtr_, i).configurationSpace();
        *configSpaceRnxSOn_ *= Joint(weakPtr_, i).RnxSOnConfigurationSpace();
      }
      if (extraConfigSpace_.dimension() > 0) {
        LiegroupSpacePtr_t extra = LiegroupSpace::create (extraConfigSpace_.dimension());
        *configSpace_       *= extra;
        *configSpaceRnxSOn_ *= extra;
      }

      numberDeviceData(numberDeviceData());
    }

    Configuration_t Device::
    neutralConfiguration () const
    {
      const Model& m (model());
      Configuration_t n (configSize());
      ::pinocchio::neutral (m, n.head(m.nq));
      n.tail(extraConfigSpace_.dimension()).setZero();
      return n;
    }

    void Device::
    addJointConstraint (JointLinearConstraint constraint)
    {
      assert (constraint.joint && constraint.reference);
      assert (constraint.joint    ->configSize() == 1);
      assert (constraint.reference->configSize() == 1);

      jointConstraints_.push_back (constraint);
    }

    std::ostream& Device::
    print (std::ostream& os) const
    {
      os << model() << iendl;
      if (jointConstraints_.size()>0)
        os << "Joint linear constraints:" << incindent;
      for (std::size_t i = 0; i < jointConstraints_.size(); ++i)
        os << iendl
          << jointConstraints_[i].joint->name() << " = "
          << jointConstraints_[i].multiplier << " * "
          << jointConstraints_[i].reference->name() << " + "
          << jointConstraints_[i].offset;
      return os << decindent << std::endl;
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
      return CollisionObjectPtr_t (new CollisionObject(weakPtr_.lock(),(GeomIndex)i));
    }

    bool Device::collisionTest (const bool stopAtFirstCollision)
    {
      /* Following hpp::model API, the forward kinematics (joint placement) is
       * supposed to have already been computed. */
      updateGeometryPlacements();
      return ::pinocchio::computeCollisions(geomModel(), geomData(),stopAtFirstCollision);
    }

    void Device::computeDistances ()
    {
      /* Following hpp::model API, the forward kinematics (joint placement) is
       * supposed to have already been computed. */
      updateGeometryPlacements();
      ::pinocchio::computeDistances (geomModel(), geomData());
    }

    const DistanceResults_t& Device::distanceResults () const
    {
      return geomData().distanceResults;
    }

    /* ---------------------------------------------------------------------- */
    /* --- Bounding box ----------------------------------------------------- */
    /* ---------------------------------------------------------------------- */

    struct AABBStep : public ::pinocchio::fusion::JointUnaryVisitorBase<AABBStep>
    {
      typedef boost::fusion::vector<const Model&,
                                    Configuration_t,
                                    bool,
                                    fcl::AABB&> ArgsType;

      template<typename JointModel>
      static void algo(const ::pinocchio::JointModelBase<JointModel> & jmodel,
                       const Model& model,
                       Configuration_t q,
                       bool initializeAABB,
                       fcl::AABB& aabb)
      {
        typedef typename RnxSOnLieGroupMap::operation<JointModel>::type LG_t;
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
    void AABBStep::algo< JointModelComposite >(
        const ::pinocchio::JointModelBase< JointModelComposite > & jmodel,
        const Model& model,
        Configuration_t q,
        bool initializeAABB,
        fcl::AABB& aabb)
    {
      // TODO this should for but I did not test it.
      hppDout(warning, "Computing AABB of JointModelComposite should work but has never been tested");
      if (initializeAABB)  {
        JointModelComposite::JointDataDerived data = jmodel.createData();
        jmodel.calc(data, q);
        aabb = fcl::AABB(data.M.translation());
      }
      ::pinocchio::details::Dispatch<AABBStep>::run(jmodel.derived(), AABBStep::ArgsType(model, q, false, aabb));
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

    void replaceGeometryByConvexHull (GeomModel& gmodel,
        const std::vector<std::string>& gnames)
    {
#if HPP_FCL_VERSION_AT_LEAST(1,4,5)
      for (std::size_t i = 0; i < gnames.size(); ++i) {
        if (!gmodel.existGeometryName(gnames[i]))
          throw std::invalid_argument("Geometry " + gnames[i] + " does not "
              "exist.");
        GeomIndex gid = gmodel.getGeometryId(gnames[i]);
        GeometryObject& go = gmodel.geometryObjects[gid];
        if (go.geometry->getObjectType() == fcl::OT_BVH) {
          boost::shared_ptr<fcl::BVHModelBase> bvh =
            HPP_DYNAMIC_PTR_CAST(fcl::BVHModelBase, go.geometry);
          assert(bvh);
          bvh->buildConvexHull(false, "Qx");
          go.geometry = bvh->convex;
        }
      }
#else
      throw std::logic_error("hpp-fcl version is below 1.4.5 does not support "
          "the generation of convex hulls.");
#endif
    }

#if __cplusplus >= 201103L
    using boost::serialization::make_nvp;
    using hpp::serialization::archive_device_wrapper;

    template<typename To, typename Ti, typename UnaryOp>
    inline std::vector<To> serialize_to (const std::vector<Ti>& in, UnaryOp op)
    {
      std::vector<To> out;
      out.reserve(in.size());
      std::transform(in.begin(), in.end(), out.begin(), op);
      return out;
    }

    template<class Archive>
    void Device::save(Archive & ar, const unsigned int version) const
    {
      (void) version;

      archive_device_wrapper* adw = dynamic_cast<archive_device_wrapper*>(&ar);
      ar & BOOST_SERIALIZATION_NVP(name_);
      bool written = (adw != NULL);
      ar & BOOST_SERIALIZATION_NVP(written);
      if (written) {
        // AbstractDevice
        ar & BOOST_SERIALIZATION_NVP(model_);
        //ar & BOOST_SERIALIZATION_NVP(geomModel_);

        // Device
        ar & BOOST_SERIALIZATION_NVP(name_);
        // - grippers_
        std::vector<FrameIndex> grippers;
        std::transform(grippers_.begin(), grippers_.end(), grippers.begin(),
            [](const GripperPtr_t& g) -> FrameIndex { return g->frameId(); });
        ar & BOOST_SERIALIZATION_NVP(grippers);
        ar & BOOST_SERIALIZATION_NVP(jointConstraints_);
        ar & BOOST_SERIALIZATION_NVP(weakPtr_);

        ar & BOOST_SERIALIZATION_NVP(extraConfigSpace_.dimension_);
        ar & BOOST_SERIALIZATION_NVP(extraConfigSpace_.lowerBounds_);
        ar & BOOST_SERIALIZATION_NVP(extraConfigSpace_.upperBounds_);

        size_type nbDeviceData = numberDeviceData();
        ar & BOOST_SERIALIZATION_NVP(nbDeviceData);
      }
    }

    template<class Archive>
    void Device::load(Archive & ar, const unsigned int version)
    {
      (void) version;
      ar & BOOST_SERIALIZATION_NVP(name_);
      bool written;
      ar & BOOST_SERIALIZATION_NVP(written);

      archive_device_wrapper* adw = dynamic_cast<archive_device_wrapper*>(&ar);
      if (written) {
        // AbstractDevice
        ar & BOOST_SERIALIZATION_NVP(model_);
        //ar & BOOST_SERIALIZATION_NVP(geomModel_);

        // Device
        ar & BOOST_SERIALIZATION_NVP(name_);
        // - grippers_
        std::vector<FrameIndex> grippers;
        ar & BOOST_SERIALIZATION_NVP(grippers);
        ar & BOOST_SERIALIZATION_NVP(jointConstraints_);
        ar & BOOST_SERIALIZATION_NVP(weakPtr_);

        ar & BOOST_SERIALIZATION_NVP(extraConfigSpace_.dimension_);
        ar & BOOST_SERIALIZATION_NVP(extraConfigSpace_.lowerBounds_);
        ar & BOOST_SERIALIZATION_NVP(extraConfigSpace_.upperBounds_);

        size_type nbDeviceData;
        ar & BOOST_SERIALIZATION_NVP(nbDeviceData);

        if (!adw) { // if archive_device_wrapper, then this device will be
          // thrown away. No need to initialize it cleanly.
          grippers_.reserve(grippers.size());
          std::transform(grippers.begin(), grippers.end(), grippers_.begin(),
              [this](FrameIndex i) -> GripperPtr_t {
              return Gripper::create (model_->frames[i].name, weakPtr_);
              });
          createData();
          createGeomData();
          numberDeviceData(nbDeviceData);
        }
      } else if (!adw) // && !written
        throw std::logic_error("This archive does not contain a valid Device "
            "and the archive is not of type archive_device_wrapper.");
      // else TODO if (adw->device->name() != name_) ?
    }
#else
    template<class Archive>
    void Device::save(Archive &, const unsigned int) const
    {
      throw std::logic_error("Not implemented without C++ 11.");
    }

    template<class Archive>
    void Device::load(Archive &, const unsigned int)
    {
      throw std::logic_error("Not implemented without C++ 11.");
    }
#endif

    HPP_SERIALIZATION_SPLIT_IMPLEMENT(Device);
  } // namespace pinocchio
} // namespace hpp

namespace boost {
namespace serialization {
template<class Archive>
void serialize (Archive & ar, hpp::pinocchio::Device::JointLinearConstraint& c, const unsigned int version)
{
  (void) version;
  ar & make_nvp("offset", c.offset);
  ar & make_nvp("multiplier", c.multiplier);
  ar & make_nvp("joint", c.joint);
  ar & make_nvp("reference", c.reference);
}
}
}
