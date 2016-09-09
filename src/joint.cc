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

# include <hpp/pinocchio/joint.hh>

# include <pinocchio/algorithm/jacobian.hpp>

# include <hpp/pinocchio/device.hh>
# include <hpp/pinocchio/body.hh>

# define CALL_JOINT(method, valueIfZero) \
  (jointIndex > 0 ? model().joints[jointIndex].method() : valueIfZero);
namespace hpp {
  namespace pinocchio {
    Joint::Joint (DevicePtr_t device, JointIndex indexInJointList ) 
      :devicePtr(device)
      ,jointIndex(indexInJointList)
    {
      assert (devicePtr);
      assert (devicePtr->modelPtr());
      assert (int(jointIndex)<model().njoint);
      setChildList();
      computeMaximalDistanceToParent();
    }

    void Joint::setChildList()
    {
      assert(devicePtr->modelPtr()); assert(devicePtr->dataPtr());
      children.clear();
      for( JointIndex child=jointIndex+1;int(child)<=data().lastChild[jointIndex];++child )
        if( model().parents[child]==jointIndex ) children.push_back (child) ;
    }

    void Joint::selfAssert() const 
    {
      assert(devicePtr);
      assert(devicePtr->modelPtr()); assert(devicePtr->dataPtr());
      assert(devicePtr->model().njoint>int(jointIndex));
    }

    Model&        Joint::model()       { selfAssert(); return devicePtr->model(); }
    const Model&  Joint::model() const { selfAssert(); return devicePtr->model(); }
    Data &        Joint::data()        { selfAssert(); return devicePtr->data (); }
    const Data &  Joint::data()  const { selfAssert(); return devicePtr->data (); }
    

    JointPtr_t Joint::parentJoint () const{
        JointIndex idParent = model().parents[jointIndex];
        if(idParent == 0)
            return JointPtr_t();
        else{
            Joint* jPtr = new Joint(devicePtr,idParent);
            return JointPtr_t(jPtr);
        }
    }


    const std::string&  Joint::name() const 
    {
      selfAssert();
      return model().names[jointIndex];
    }

    const Transform3f&  Joint::currentTransformation () const 
    {
      selfAssert();
      return data().oMi[jointIndex];
    }

    size_type  Joint::numberDof () const 
    {
      selfAssert();
      return CALL_JOINT(nv, 0);
    }
    size_type  Joint::configSize () const
    {
      selfAssert();
      return CALL_JOINT(nq, 0);
    }

    size_type  Joint::rankInConfiguration () const
    {
      selfAssert();
      return CALL_JOINT(idx_q, 0);
    }

    size_type  Joint::rankInVelocity () const
    {
      selfAssert();
      return CALL_JOINT(idx_v, 0);
    }

    std::size_t  Joint::numberChildJoints () const
    {
      return children.size();
    }

    JointPtr_t  Joint::childJoint (std::size_t rank) const
    {
      selfAssert();
      assert(rank<children.size());
      return JointPtr_t (new Joint(devicePtr,children[rank]) );
    }

    const Transform3f&  Joint::positionInParentFrame () const
    {
      selfAssert();
      return model().jointPlacements[jointIndex];
    }

    void Joint::positionInParentFrame (const Transform3f& p)
    {
      selfAssert();
      model().jointPlacements[jointIndex] = p;
    }

    void Joint::isBounded (size_type rank, bool bounded)
    {
      const size_type idx = model().joints[jointIndex].idx_q() + rank;
      assert(rank < configSize());
      if (!bounded) {
        const value_type& inf = std::numeric_limits<value_type>::infinity();
        model().lowerPositionLimit[idx] = -inf;
        model().upperPositionLimit[idx] =  inf;
      } else {
        assert(false && "This function can only unset bounds. "
           "Use lowerBound and upperBound to set the bounds.");
      }
    }
    bool Joint::isBounded (size_type rank) const
    {
      const size_type idx = model().joints[jointIndex].idx_q() + rank;
      const value_type& inf = std::numeric_limits<value_type>::infinity();
      assert(rank < configSize());
      return !( model().lowerPositionLimit[idx] == -inf)
        ||   !( model().upperPositionLimit[idx] ==  inf);
    }
    value_type Joint::lowerBound (size_type rank) const
    {
      const size_type idx = model().joints[jointIndex].idx_q() + rank;
      assert(rank < configSize());
      return model().lowerPositionLimit[idx];
    }
    value_type Joint::upperBound (size_type rank) const
    {
      const size_type idx = model().joints[jointIndex].idx_q() + rank;
      assert(rank < configSize());
      return model().upperPositionLimit[idx];
    }
    void Joint::lowerBound (size_type rank, value_type lowerBound)
    {
      const size_type idx = model().joints[jointIndex].idx_q() + rank;
      assert(rank < configSize());
      model().lowerPositionLimit[idx] = lowerBound;
    }
    void Joint::upperBound (size_type rank, value_type upperBound)
    {
      const size_type idx = model().joints[jointIndex].idx_q() + rank;
      assert(rank < configSize());
      model().upperPositionLimit[idx] = upperBound;
    }


    /* --- MAX DISTANCE ------------------------------------------------------*/
    /* --- MAX DISTANCE ------------------------------------------------------*/
    /* --- MAX DISTANCE ------------------------------------------------------*/

    template <bool X, bool Y, bool Z, typename Derived>
    value_type computeMaximalDistanceToParentForAlignedTranslation(
        const Eigen::MatrixBase<Derived>& lower,
        const Eigen::MatrixBase<Derived>& upper,
        const se3::SE3& placement)
    {
      if (!lower.allFinite() || !upper.allFinite())
        return std::numeric_limits <value_type>::infinity ();

      value_type d = 0;
      const size_type iX = 0;
      const size_type iY = (X ? 1 : 0);
      const size_type iZ = iY + (Y ? 1 : 0);
      vector3_t p (vector3_t::Zero());
      for (size_type i = 0; i < (X ? 1 : 2); ++i) {
        if (X) p[0] = (i == 0 ? lower[iX] : upper[iX]);
        for (size_type j = 0; j < (Y ? 1 : 2); ++j) {
          if (Y) p[1] = (j == 0 ? lower[iY] : upper[iY]);
          for (size_type k = 0; k < (Z ? 1 : 2); ++k) {
            if (Z) p[1] = (k == 0 ? lower[iZ] : upper[iZ]);
            d = std::max(d, placement.act(p).norm());
          }
        }
      }
      return d;
    }

    template<typename Joint>
    value_type computeMaximalDistanceToParent( const se3::Model & model,
                                               const se3::JointModelBase<Joint> & ,
                                               const se3::SE3 & jointPlacement )
    {
      assert (false 
              && "The function <maximalDistance> as not been implemented for this class of joint");
      return 0.0;
    }

    value_type computeMaximalDistanceToParent
    ( const se3::Model & model, const se3::JointModelFreeFlyer& jmodel, const se3::SE3 & jointPlacement )
    {
      const size_type& i = jmodel.idx_q();
      return computeMaximalDistanceToParentForAlignedTranslation<true, true, true>(
          model.lowerPositionLimit.segment<se3::JointModelTranslation::NQ>(i),
          model.upperPositionLimit.segment<se3::JointModelTranslation::NQ>(i),
          jointPlacement);
    }

    template<int AXIS>
    value_type computeMaximalDistanceToParent
    ( const se3::Model & model, const se3::JointModelRevolute<AXIS> & , const se3::SE3 & jointPlacement )
    { return jointPlacement.translation().norm(); }

    value_type computeMaximalDistanceToParent
    ( const se3::Model & model, const se3::JointModelRevoluteUnaligned &, const se3::SE3 & jointPlacement )
    { return jointPlacement.translation().norm(); }

    template<int AXIS>
    value_type computeMaximalDistanceToParent
    ( const se3::Model & model, const se3::JointModelRevoluteUnbounded<AXIS> & , 
      const se3::SE3 & jointPlacement )
    { return jointPlacement.translation().norm(); }

    template<int AXIS>
    value_type computeMaximalDistanceToParent
    ( const se3::Model & model, const se3::JointModelPrismatic<AXIS> & jmodel, 
      const se3::SE3 & jointPlacement )
    {
      return computeMaximalDistanceToParentForAlignedTranslation
        <AXIS == 0, AXIS == 1, AXIS == 2>(
          model.lowerPositionLimit.segment<1>(jmodel.idx_q()),
          model.upperPositionLimit.segment<1>(jmodel.idx_q()),
          jointPlacement);
    }

    value_type computeMaximalDistanceToParent
    ( const se3::Model & model, const se3::JointModelPrismaticUnaligned& jmodel, 
      const se3::SE3 & jointPlacement )
    {
      if( std::isinf (model.lowerPositionLimit[jmodel.nq()])
          || std::isinf (model.upperPositionLimit[jmodel.nq()]) )
        return std::numeric_limits <value_type>::infinity (); 
 
      Eigen::Vector3d pmin = jmodel.axis *  model.lowerPositionLimit[jmodel.nq()];
      Eigen::Vector3d pmax = jmodel.axis *  model.upperPositionLimit[jmodel.nq()];

      return std::max ( jointPlacement.act(pmin).norm(),
                        jointPlacement.act(pmax).norm() );
    }

    value_type computeMaximalDistanceToParent
    ( const se3::Model & model, const se3::JointModelSpherical& , const se3::SE3 & jointPlacement )
    { return jointPlacement.translation().norm(); }

    value_type computeMaximalDistanceToParent
    ( const se3::Model & model, const se3::JointModelSphericalZYX& , const se3::SE3 & jointPlacement )
    { return jointPlacement.translation().norm(); }

    value_type computeMaximalDistanceToParent
    ( const se3::Model & model, const se3::JointModelTranslation& jmodel, const se3::SE3 & jointPlacement )
    {
      const size_type& i = jmodel.idx_q();
      return computeMaximalDistanceToParentForAlignedTranslation<true, true, true>(
          model.lowerPositionLimit.segment<se3::JointModelTranslation::NQ>(i),
          model.upperPositionLimit.segment<se3::JointModelTranslation::NQ>(i),
          jointPlacement);
    }
    // TODO (really?): handle the case where the translation is bounded.

    value_type computeMaximalDistanceToParent
    ( const se3::Model & model, const se3::JointModelPlanar& jmodel, const se3::SE3 & jointPlacement )
    {
      const size_type& i = jmodel.idx_q();
      return computeMaximalDistanceToParentForAlignedTranslation <true, true, false> (
          model.lowerPositionLimit.segment<2>(i),
          model.upperPositionLimit.segment<2>(i),
          jointPlacement);
    }

    struct VisitMaximalDistanceToParent : public boost::static_visitor<value_type> 
    {
      const se3::Model & model;
      const se3::SE3 & jointPlacement;
      VisitMaximalDistanceToParent( const se3::Model & model, const se3::SE3 & jointPlacement )
        : model(model), jointPlacement(jointPlacement) {}

      template<typename Joint>
      value_type operator() ( const se3::JointModelBase<Joint> & jmodel )
      { return computeMaximalDistanceToParent(model,jmodel.derived(),jointPlacement) ; }
    };

    void  Joint::computeMaximalDistanceToParent () 
    {
      VisitMaximalDistanceToParent visitor(model(),
                                           model().jointPlacements[jointIndex]);
      const se3::JointModelVariant & jmv = model().joints[jointIndex];
      maximalDistanceToParent_ = 
        boost::apply_visitor( visitor, jmv );
    }

    /* --- MAX VEL -----------------------------------------------------------*/
    /* --- MAX VEL -----------------------------------------------------------*/
    /* --- MAX VEL -----------------------------------------------------------*/

    /* --- LINEAR VELOCITY ---------------------------------------------------*/
    template<typename D>
    value_type upperBoundLinearVelocity( const se3::JointModelBase<D> & )                  
    {
      assert (false 
              && "The function <upperBoundLinearVel> as not been implemented for this class of joint");
      return 0.0;
    }
    value_type upperBoundLinearVelocity( const se3::JointModelFreeFlyer & )                  { return 1.0; }
    template<int AXIS>
    value_type upperBoundLinearVelocity( const se3::JointModelRevolute<AXIS> & )             { return 0.0; }
    value_type upperBoundLinearVelocity( const se3::JointModelRevoluteUnaligned & )          { return 0.0; }
    template<int AXIS>
    value_type upperBoundLinearVelocity( const se3::JointModelRevoluteUnbounded<AXIS> & )    { return 0.0; }
    template<int AXIS>
    value_type upperBoundLinearVelocity( const se3::JointModelPrismatic<AXIS> & )            { return 1.0; }
    value_type upperBoundLinearVelocity( const se3::JointModelPrismaticUnaligned & )         { return 1.0; }
    value_type upperBoundLinearVelocity( const se3::JointModelSpherical & )                  { return 0.0; }
    value_type upperBoundLinearVelocity( const se3::JointModelSphericalZYX & )               { return 0.0; }
    value_type upperBoundLinearVelocity( const se3::JointModelTranslation & )                { return 1.0; }
    value_type upperBoundLinearVelocity( const se3::JointModelPlanar & )                     { return 1.0; }


    struct VisitUpperBoundLinearVelocity : public boost::static_visitor<value_type> 
    {
      template<typename Joint>
      value_type operator() ( const se3::JointModelBase<Joint> & jmodel ) const
      { return upperBoundLinearVelocity(jmodel.derived()) ; }
    };

    value_type  Joint::upperBoundLinearVelocity () const
    {
      VisitUpperBoundLinearVelocity visitor;
      const se3::JointModelVariant & jmv = model().joints[jointIndex];

      //return boost::apply_visitor(visitor,jmv);
      return boost::apply_visitor(VisitUpperBoundLinearVelocity(),jmv);
    }
 
    /* --- ANGULAR VELOCITY -------------------------------------------------- */
    template<typename D>
    value_type upperBoundAngularVelocity( const se3::JointModelBase<D> & )                  
    {
      assert (false 
              && "The function <upperBoundAngularVel> as not been implemented for this class of joint");
      return 0.0;
    }
    value_type upperBoundAngularVelocity( const se3::JointModelFreeFlyer & )                  { return 1.0; }
    template<int AXIS>
    value_type upperBoundAngularVelocity( const se3::JointModelRevolute<AXIS> & )             { return 1.0; }
    value_type upperBoundAngularVelocity( const se3::JointModelRevoluteUnaligned & )          { return 1.0; }
    template<int AXIS>
    value_type upperBoundAngularVelocity( const se3::JointModelRevoluteUnbounded<AXIS> & )    { return 1.0; }
    template<int AXIS>
    value_type upperBoundAngularVelocity( const se3::JointModelPrismatic<AXIS> & )            { return 0.0; }
    value_type upperBoundAngularVelocity( const se3::JointModelPrismaticUnaligned & )         { return 0.0; }
    value_type upperBoundAngularVelocity( const se3::JointModelSpherical & )                  { return 1.0; }
    value_type upperBoundAngularVelocity( const se3::JointModelSphericalZYX & )               { return 1.0; }
    value_type upperBoundAngularVelocity( const se3::JointModelTranslation & )                { return 0.0; }
    value_type upperBoundAngularVelocity( const se3::JointModelPlanar & )                     { return 1.0; }

    struct VisitUpperBoundAngularVelocity : public boost::static_visitor<value_type> 
    {
      template<typename Joint>
      value_type operator() ( const se3::JointModelBase<Joint> & jmodel ) const
      { return upperBoundAngularVelocity(jmodel.derived()) ; }
    };

    value_type  Joint::upperBoundAngularVelocity () const
    {
      VisitUpperBoundAngularVelocity visitor;
      const se3::JointModelVariant & jmv = model().joints[jointIndex];

      //return boost::apply_visitor(visitor,jmv);
      return boost::apply_visitor(VisitUpperBoundAngularVelocity(),jmv);
    }

    const JointJacobian_t&  Joint::jacobian (const bool local) const
    {
      selfAssert(); assert(robot()->computationFlag() & Device::JACOBIAN);
      if( jacobian_.cols()!=model().nv)  jacobian_ = JointJacobian_t::Zero(6,model().nv);
      if(local) se3::getJacobian<true> (model(),data(),jointIndex,jacobian_);
      else      se3::getJacobian<false>(model(),data(),jointIndex,jacobian_);
      return jacobian_;
    }

    JointJacobian_t&  Joint::jacobian (const bool local)
    {
      selfAssert(); assert(robot()->computationFlag() & Device::JACOBIAN);
      if( jacobian_.cols()!=model().nv)  jacobian_ = JointJacobian_t::Zero(6,model().nv);
      if(local) se3::getJacobian<true> (model(),data(),jointIndex,jacobian_);
      else      se3::getJacobian<false>(model(),data(),jointIndex,jacobian_);
      return jacobian_;
    }

    BodyPtr_t  Joint::linkedBody () const 
    {
      return BodyPtr_t( new Body(devicePtr,jointIndex) );
    }

    std::ostream& Joint::display (std::ostream& os) const 
    {
      os << "Joint " << jointIndex 
         << "(nq=" << configSize() << ",nv=" << numberDof() << ")" << std::endl;

      os << "\"" << name () << "\"" << "[shape=box label=\"" << name ()
	 << "\\n";
      if (configSize () != 0)
	os << "Rank in configuration: " << rankInConfiguration() << "\\n";
      else
	os << "Anchor joint\\n";
      os << "Current transformation: " << currentTransformation();
      os << "\\n";
      /*hpp::model::BodyPtr_t body = linkedBody();
      if (body) {
	const matrix3_t& I = body->inertiaMatrix();
	os << "Attached body: " << body->name () << "\\n";
	os << "Mass of the attached body: " << body->mass() << "\\n";
	os << "Local center of mass:" << body->localCenterOfMass() << "\\n";
	os << "Inertia matrix:" << "\\n";
	os << I (0,0) << "\t" << I (0,1) << "\t" << I (0,2) << "\\n"
	   << I (1,0) << "\t" << I (1,1) << "\t" << I (1,2) << "\\n"
	   << I (2,0) << "\t" << I (2,1) << "\t" << I (2,2) << "\\n";
	os << "geometric objects" << "\\n";
	const hpp::model::ObjectVector_t& colObjects =
	  body->innerObjects (hpp::model::COLLISION);
	for (hpp::model::ObjectVector_t::const_iterator it =
	       colObjects.begin (); it != colObjects.end (); ++it) {
	  os << "name: " << (*it)->name () << "\\n";
	  os << "position in joint:" << "\\n";
	  const fcl::Transform3f& local ((*it)->positionInJointFrame ());
	  displayTransform3f (os, local); os << "\\n";
	  os << "position :" << "\\n";
	  const fcl::Transform3f& global ((*it)->fcl ()->getTransform ());
	  displayTransform3f (os, global);
	}
      } else {
	os << "No body";
        }*/
      os << "\"]" << std::endl;
      for (unsigned int iChild=0; iChild < numberChildJoints (); iChild++)
        {
          // for (hpp::model::JointVector_t::const_iterator it =
	  //    children_.begin (); it != children_.end (); ++it) {

          // write edges to children joints
          os << "\"" << name () << "\"->\"" << childJoint(iChild)->name () << "\""
             << std::endl;
      }
      return os;
    }

    const se3::JointModel& Joint::jointModel() const
    {
      return model().joints[index()];
    }

  } // namespace pinocchio
} // namespace hpp
