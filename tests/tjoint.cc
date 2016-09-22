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

#define BOOST_TEST_MODULE tjoint

#include <boost/test/unit_test.hpp>

#include <hpp/model/device.hh>
#include <hpp/model/urdf/util.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include "../tests/utils.hh"

static bool verbose = false;

/* -------------------------------------------------------------------------- */

//      Joint (DeviceWkPtr_t device, Index indexInJointList );
//      ~Joint() {}
//      const std::string& name() const;
//      const Transform3f& currentTransformation () const;
//      size_type numberDof () const;
//      size_type configSize () const;
//      size_type rankInConfiguration () const;
//      size_type rankInVelocity () const;
//      std::size_t numberChildJoints () const;
//      JointPtr_t childJoint (std::size_t rank) const;
//      const Transform3f& positionInParentFrame () const;
//      value_type upperBoundLinearVelocity () const;
//      value_type upperBoundAngularVelocity () const;
//      const value_type& maximalDistanceToParent () const;
//      void computeMaximalDistanceToParent ();
//      const JointJacobian_t& jacobian () const;
//      JointJacobian_t& jacobian ();
//      DeviceConstPtr_t robot () const { assert(robot_.lock());  return robot_.lock ();}
//      DevicePtr_t robot () { assert(robot_.lock()); return robot_.lock ();}
//      BodyPtr_t linkedBody () const;
//      virtual std::ostream& display (std::ostream& os) const;

/* -------------------------------------------------------------------------- */
/*
 * Check whether joint <jp> has a child named <name>. 
 * Search is exhaustive among the children ( cost o(nchild) ).
 */
static bool hasChild (hpp::pinocchio::JointPtr_t jp,std::string name)
{
  //std::cout << "father name " << jp->name() << " nc=" << jp->numberChildJoints() << std::endl;
  for( unsigned int c=0;c<jp->numberChildJoints();++c )
    {
      //std::cout <<"child name " << jp->childJoint(c)->name() << std::endl;
      if( jp->childJoint(c)->name() == name ) return true;
    }
  return false;
}

/* --- UNIT TESTS ----------------------------------------------------------- */
/* --- UNIT TESTS ----------------------------------------------------------- */
/* --- UNIT TESTS ----------------------------------------------------------- */

BOOST_AUTO_TEST_CASE (joint)
{
  hpp::model::DevicePtr_t     model     = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  Eigen::VectorXd q = Eigen::VectorXd::Random( model->configSize() );
  q[3] += 1.0;
  q.segment<4>(3) /= q.segment<4>(3).norm();

  model->currentConfiguration(q);
  model->controlComputation (hpp::model::Device::ALL);
  model->computeForwardKinematics();

  pinocchio->currentConfiguration(m2p::q(q));
  pinocchio->controlComputation (hpp::pinocchio::Device::ALL);
  pinocchio->computeForwardKinematics();  

  if( verbose )
    {
      for( int i=1;i<8;++i)
        {
          std::cout << " *************************************************** " << std::endl;
          std::cout << "model = " << model->getJointVector()[i+1]->name() << std::endl;
          std::cout << "pino = " << pinocchio->model().names[i] << std::endl;
          std::cout << "type = " << pinocchio->model().joints[i].shortname() << std::endl;
          std::cout << typeid(model->getJointVector()[i+1]).name() << std::endl;
          
          std::cout << "5M6n = " << model->getJointVector()[i+1]->positionInParentFrame() << std::endl;
          std::cout << "5M6n = " << pinocchio->model().jointPlacements[i] << std::endl;
          
          std::cout << "oM6n = " << model->getJointVector()[i+1]->currentTransformation() << std::endl;
          std::cout << "oM6n = " << pinocchio->data().oMi[i] << std::endl;
        }
    }

  pinocchio->model().names[1] = "waist";

  for( unsigned int i=2;i<model->getJointVector().size();++i)
    {
      hpp::model::JointPtr_t jm = model->getJointVector()[i];
      if(verbose)std::cout << " ***************** " << jm->name()<< std::endl;
      if(dynamic_cast<hpp::model::JointAnchor *> (jm)) // is an anchor
        continue;

      hpp::pinocchio::JointPtr_t jp;
      try
        { jp = pinocchio->getJointByName(jm->name()); }
      catch (...)
        { BOOST_CHECK( false && "Joint name does not exist" ); }
      
      BOOST_CHECK( jp->name()                == jm->name() );
      BOOST_CHECK( jp->numberDof()           == jm->numberDof() );
      BOOST_CHECK( jp->configSize()          == jm->configSize() );
      BOOST_CHECK( jp->rankInConfiguration() == jm->rankInConfiguration() );
      BOOST_CHECK( jp->rankInVelocity()      == jm->rankInVelocity() );
      BOOST_CHECK( isApproxPermutation(jm->currentTransformation(),
                                       jp->currentTransformation()) );

      for( unsigned int c=0; c<jm->configSize(); ++c ) {
        BOOST_CHECK(jp->isBounded(c) == jm->isBounded(c));
      }
      using hpp::pinocchio::value_type;
      jp->upperBound(0,  std::numeric_limits<value_type>::infinity());
      jp->lowerBound(0, -std::numeric_limits<value_type>::infinity());
      BOOST_CHECK_MESSAGE(!jp->isBounded(0),
          "Wrong isBounded result: " << jp->lowerBound(0) << ", " << jp->upperBound(0)
          );
      jp->upperBound(0,  0                                          );
      jp->lowerBound(0, -std::numeric_limits<value_type>::infinity());
      BOOST_CHECK_MESSAGE(!jp->isBounded(0),
          "Wrong isBounded result: " << jp->lowerBound(0) << ", " << jp->upperBound(0)
          );
      jp->upperBound(0,  0                                          );
      jp->lowerBound(0, -1                                          );
      BOOST_CHECK_MESSAGE( jp->isBounded(0),
          "Wrong isBounded result: " << jp->lowerBound(0) << ", " << jp->upperBound(0)
          );
      
      // Possibly some children are anchor, and then removed from Pinocchio
      BOOST_CHECK( jp->numberChildJoints()   <= jm->numberChildJoints() );
      for( unsigned int c=0; c<jm->numberChildJoints(); ++c )
        {
          hpp::model::JointPtr_t cm = jm->childJoint(c);
          if(dynamic_cast<hpp::model::JointAnchor *> (cm)) // is not an anchor
            continue;
          BOOST_CHECK ( hasChild(jp,cm->name()) );
        }
      
      BOOST_CHECK( jp->robot() == pinocchio );
      BOOST_CHECK( jm->robot() == model );
      
      Eigen::MatrixXd Jm = jm->jacobian();
      Eigen::MatrixXd Jp = jp->jacobian();
      se3::SE3 oMb = pinocchio->data().oMi[1];
      se3::SE3 oMe = jp->currentTransformation();

      if((i==8) && verbose)
        {
          std::cout << "Jm  = [ " << Jm.leftCols(9)                 << "] ;" << std::endl;
          std::cout << "Jp  = [ " << Jp.leftCols(9)                 << "] ;" << std::endl;
          std::cout << "oMb = [ " << oMb.toHomogeneousMatrix()      << "] ;" << std::endl;
          std::cout << "oMe = [ " << oMe.toHomogeneousMatrix()      << "] ;" << std::endl;
        }

      BOOST_CHECK( (p2m::X(oMe)*Jp*m2p::Xq(oMb)).isApprox(Jm) );
      BOOST_CHECK( (m2p::X(oMe)*Jm*p2m::Xq(oMb)).isApprox(Jp) );

      BOOST_CHECK( std::abs(jp->maximalDistanceToParent()  -jm->maximalDistanceToParent())  <1e-6 );
      BOOST_CHECK( std::abs(jp->upperBoundLinearVelocity() -jm->upperBoundLinearVelocity()) <1e-6 );
      BOOST_CHECK( std::abs(jp->upperBoundAngularVelocity()-jm->upperBoundAngularVelocity())<1e-6 );
    }

  /* Checking positionInParentFrame is difficult because of the modification of revolute
   * joints in hpp::model (all oriented along X). Then only check the two first ones, where
   * we can confidently assume that they are properly aligned. */
  BOOST_CHECK( isApproxPermutation( model    ->getJointByName("waist")->positionInParentFrame(),
                                    pinocchio->getJointByName("waist")->positionInParentFrame() ) );
  /* The first two children are IMU spots. The hip is in rank=2 in hpp::model.  
   * So check model.child(2) against pinocchio.child(0). */
  BOOST_CHECK( isApproxPermutation( model    ->getJointByName("waist")
                                    ->childJoint(2)->positionInParentFrame(),
                                    pinocchio->getJointByName("waist")
                                    ->childJoint(0)->positionInParentFrame() ) );
}

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (jointVector)
{
  hpp::model::DevicePtr_t     model     = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  hpp::pinocchio::JointVector jv(pinocchio);
  {
    hpp::pinocchio::JointVector::iterator it = jv.begin();
    BOOST_CHECK( (*it)->name() == pinocchio->rootJoint()->name() );
  }

  /* Check regular iterator, skipping root joint (names are different). */
  int check = 1;
  for (hpp::pinocchio::JointVector::iterator it = ++jv.begin (); it != jv.end (); ++it) 
    {
      hpp::model::JointPtr_t jm;
      do
        {
          jm = model->getJointVector()[++check];
        } while(dynamic_cast<hpp::model::JointAnchor *> (jm)); // is not an anchor

      BOOST_CHECK (jm->name() == (*it)->name());
      // std::cout << "\"" << jm->name()
      //           << "\"->\"" << (*it)->name () << "\"" << std::endl;
    }

  /* Check reverse iterator. */
  check = model->getJointVector().size()-1;
  for (hpp::pinocchio::JointVector::iterator it = jv.rbegin (); it != ++jv.rend (); --it) 
    {
      hpp::model::JointPtr_t jm;
      do
        {
          jm = model->getJointVector()[check]; --check;
        } while(dynamic_cast<hpp::model::JointAnchor *> (jm)); // is not an anchor

      BOOST_CHECK (jm->name() == (*it)->name());
      //std::cout << "\"" << jm->name()
      //<< "\"->\"" << (*it)->name () << "\"" << std::endl;
    }

  /* Check constant iterator. */
  const hpp::pinocchio::JointVector jvconst(pinocchio);
  check = 1;
  for (hpp::pinocchio::JointVector::const_iterator it = ++jvconst.begin (); it != jv.end (); ++it) 
    {
      hpp::model::JointPtr_t jm;
      do
        {
          jm = model->getJointVector()[++check];
        } while(dynamic_cast<hpp::model::JointAnchor *> (jm)); // is not an anchor

      BOOST_CHECK (jm->name() == (*it)->name());
    }

}
