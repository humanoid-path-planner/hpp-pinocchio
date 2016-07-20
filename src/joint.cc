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
# include <hpp/pinocchio/device.hh>
# include <hpp/pinocchio/body.hh>
# include <pinocchio/algorithm/jacobian.hpp>

namespace hpp {
  namespace pinocchio {
    Joint::Joint (DevicePtr_t device, Index indexInJointList ) 
      :devicePtr(device)
      ,jointIndex(indexInJointList)
    {
      assert (model());
      assert (int(jointIndex)<model()->njoint);
      setChildList();
    }

    void Joint::setChildList()
    {
      assert(model()); assert(data());
      children.clear();
      for( se3::Index child=jointIndex+1;int(child)<=data()->lastChild[jointIndex];++child )
        if( model()->parents[child]==jointIndex ) children.push_back (child) ;
    }

    void Joint::selfAssert() const 
    {
      assert(devicePtr);
      assert(model()); assert(data());
      assert(devicePtr->model()->njoint>int(jointIndex));
    }

    ModelPtr_t       Joint::model()       { DevicePtr_t r = robot(); return r->model(); }
    ModelConstPtr_t  Joint::model() const { return devicePtr->model(); }
    DataPtr_t        Joint::data()        { return devicePtr->data (); }
    DataConstPtr_t   Joint::data()  const { return devicePtr->data (); }

    const std::string&  Joint::name() const 
    {
      selfAssert();
      return model()->names[jointIndex];
    }

    const Transform3f&  Joint::currentTransformation () const 
    {
      selfAssert();
      return data()->oMi[jointIndex];
    }

//NOTYET    vector_t  Joint::neutralConfiguration () const {}

    size_type  Joint::numberDof () const 
    {
      selfAssert();
      return model()->joints[jointIndex].nv();
    }
    size_type  Joint::configSize () const
    {
      selfAssert();
      return model()->joints[jointIndex].nq();
    }

    size_type  Joint::rankInConfiguration () const
    {
      selfAssert();
      return model()->joints[jointIndex].idx_q();
    }

    size_type  Joint::rankInVelocity () const
    {
      selfAssert();
      return model()->joints[jointIndex].idx_v();
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
      return model()->jointPlacements[jointIndex];
    }

//NOTYET    value_type  Joint::upperBoundLinearVelocity () const {}
//NOTYET    value_type  Joint::upperBoundAngularVelocity () const {}
//NOTYET    const value_type& m Joint::aximalDistanceToParent () const {}
//NOTYET    void  Joint::computeMaximalDistanceToParent () {}

    const JointJacobian_t&  Joint::jacobian (const bool local) const
    {
      selfAssert(); assert(robot()->computationFlag()|Device::JACOBIAN);
      if(local) se3::getJacobian<true> (*model(),*data(),jointIndex,jacobian_);
      else      se3::getJacobian<false>(*model(),*data(),jointIndex,jacobian_);
      return jacobian_;
    }

    JointJacobian_t&  Joint::jacobian (const bool local)
    {
      selfAssert(); assert(robot()->computationFlag()|Device::JACOBIAN);
      if(local) se3::getJacobian<true> (*model(),*data(),jointIndex,jacobian_);
      else      se3::getJacobian<false>(*model(),*data(),jointIndex,jacobian_);
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

    /* --- ITERATOR --------------------------------------------------------- */
    
    /* Access to pinocchio index + 1 because pinocchio first joint is the universe. */
    JointPtr_t JointVector::at(const size_type i) 
    { selfAssert(i); return JointPtr_t(new Joint(devicePtr,i+1)); }
    
    /* Access to pinocchio index + 1 because pinocchio first joint is the universe. */
    JointConstPtr_t JointVector::at(const size_type i) const 
    { selfAssert(i); return JointConstPtr_t(new Joint(devicePtr,i+1)); }

    size_type JointVector::size() const 
    { return devicePtr->model()->njoint - 1; }

    size_type JointVector::iend() const 
    { return size(); }

    void JointVector::selfAssert(size_type i) const
    {
      assert(devicePtr);
      assert(i>=ibegin());
      assert(i<iend());
    }

  } // namespace pinocchio
} // namespace hpp
