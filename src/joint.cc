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

namespace hpp {
  namespace pinocchio {
    Joint::Joint (DeviceWkPtr_t device, Index indexInJointList ) 
      :robot_(device)
      ,id(indexInJointList)
    {
      assert (model());
      assert (int(id)<model()->njoint);
      setChildList();
    }

    void Joint::setChildList()
    {
      assert(model()); assert(data());
      children.clear();
      for( se3::Index child=id+1;int(child)<=data()->lastChild[id];++child )
        if( model()->parents[child]==id ) children.push_back (child) ;
    }

    ModelPtr_t       Joint::model()       { DevicePtr_t r = robot(); return r->model(); }
    ModelConstPtr_t  Joint::model() const { return robot()->model(); }
    DataPtr_t        Joint::data()        { return robot()->data (); }
    DataConstPtr_t   Joint::data()  const { return robot()->data (); }

    const std::string&  Joint::name() const 
    {
      assert (model());
      return model()->names[id];
    }

    const Transform3f&  Joint::currentTransformation () const 
    {
      assert(data());
      return data()->oMi[id];
    }

//NOTYET    vector_t  Joint::neutralConfiguration () const {}

    size_type  Joint::numberDof () const 
    {
      assert(model());
      return model()->joints[id].nv();
    }
    size_type  Joint::configSize () const
    {
      assert (model());
      return model()->joints[id].nq();
    }

    size_type  Joint::rankInConfiguration () const
    {
      assert (model());
      return model()->joints[id].idx_q();
    }

    size_type  Joint::rankInVelocity () const
    {
      assert (model());
      return model()->joints[id].idx_v();
    }

    std::size_t  Joint::numberChildJoints () const
    {
      return children.size();
    }

    JointPtr_t  Joint::childJoint (std::size_t rank) const
    {
      assert (model()); assert(rank<children.size());
      return JointPtr_t (new Joint(robot_,children[rank]) );
    }

    const Transform3f&  Joint::positionInParentFrame () const
    {
      assert(model());
      return model()->jointPlacements[id];
    }

//NOTYET    value_type  Joint::upperBoundLinearVelocity () const {}
//NOTYET    value_type  Joint::upperBoundAngularVelocity () const {}
//NOTYET    const value_type& m Joint::aximalDistanceToParent () const {}
//NOTYET    void  Joint::computeMaximalDistanceToParent () {}
//NOTYET    const JointJacobian_t&  Joint::jacobian () const {}

//NOTYET    JointJacobian_t&  Joint::jacobian () {}
//NOTYET    BodyPtr_t  Joint::linkedBody () const {}

    std::ostream& Joint::display (std::ostream& os) const 
    {
      os << "Joint " << id 
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

    typedef JointVector::iterator iterator;
    iterator&  iterator::operator++() { ++idx; return *this; }
    iterator   iterator::operator++(int) { iterator copy = *this; idx++; return copy; }
    iterator&  iterator::operator--() { --idx; return *this; }
    iterator   iterator::operator--(int) { iterator copy = *this; idx--; return copy; }
    JointPtr_t iterator::operator* () { assert(ref.device); return JointPtr_t (new Joint(ref.device,idx)); }
    bool       iterator::operator==(const iterator & i2) { return idx==i2.idx; }
    bool       iterator::operator!=(const iterator & i2) { return idx!=i2.idx; }

    iterator JointVector::begin() { assert(device); return iterator(*this,1); }
    iterator JointVector::end()   { assert(device); return iterator(*this,device->model()->njoint); }
    iterator JointVector::rbegin(){ assert(device); return iterator(*this,device->model()->njoint-1); }
    iterator JointVector::rend()  { assert(device); return iterator(*this,0); }

    typedef JointVector::const_iterator const_iterator;
    const_iterator&  const_iterator::operator++() { ++idx; return *this; }
    const_iterator   const_iterator::operator++(int) { const_iterator copy = *this; idx++; return copy; }
    const_iterator&  const_iterator::operator--() { --idx; return *this; }
    const_iterator   const_iterator::operator--(int) { const_iterator copy = *this; idx--; return copy; }
    JointPtr_t       const_iterator::operator* () { return JointPtr_t (new Joint(ref.device,idx)); }
    bool             const_iterator::operator==(const const_iterator & i2) { return idx==i2.idx; }
    bool             const_iterator::operator!=(const const_iterator & i2) { return idx!=i2.idx; }

    const_iterator JointVector::begin() const { assert(device); return const_iterator(*this,1); }
    const_iterator JointVector::end()   const { assert(device); return const_iterator(*this,device->model()->njoint); }
    const_iterator JointVector::rbegin()const { assert(device); return const_iterator(*this,device->model()->njoint-1); }
    const_iterator JointVector::rend()  const { assert(device); return const_iterator(*this,0); }

    JointPtr_t     JointVector::operator[](const int idx) 
    {
      assert(device); 
      return JointPtr_t( new Joint(device,idx+1) ); 
    }

    const Joint*    JointVector::operator[](const int idx) const
    {
      assert(device); 
      return new Joint(device,idx+1); 
    }

  } // namespace pinocchio
} // namespace hpp
