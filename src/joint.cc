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
      for( se3::Index child=id+1;int(child)<data()->lastChild[id];++child )
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
      return os;
    }

  } // namespace pinocchio
} // namespace hpp
