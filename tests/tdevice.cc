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

#define BOOST_TEST_MODULE tdevice

#include <boost/test/unit_test.hpp>

#include <hpp/model/device.hh>
#include <hpp/model/urdf/util.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include "../tests/utils.hh"

static bool verbose = false;

//      virtual ~Device() ;
//NOTCHECKED      DevicePtr_t clone()
//NOTCHECKED      DevicePtr_t cloneConst() const
//NOTCHECKED      const std::string& name () const
//      static DevicePtr_t create (const std::string & name);
//NOTCHECKED      static DevicePtr_t createCopy (const DevicePtr_t& device);
//NOTCHECKED      static DevicePtr_t createCopyConst (const DeviceConstPtr_t& device);
//      void model( ModelPtr_t modelPtr )
//      ModelConstPtr_t model() const
//      ModelPtr_t model()
//      void data( DataPtr_t dataPtr )
//      DataConstPtr_t data() const
//      DataPtr_t data()
//      void createData();
//      const JointVector_t& getJointVector () const;
//      JointPtr_t rootJoint () const;
//      JointPtr_t getJointAtConfigRank (const size_type& r) const;
//      JointPtr_t getJointAtVelocityRank (const size_type& r) const;
//      JointPtr_t getJointByName (const std::string& name) const;
//NOTCHECKED      JointPtr_t getJointByBodyName (const std::string& name) const;
//      size_type configSize () const;
//      size_type numberDof () const;
//NOTCHECKED      ExtraConfigSpace& extraConfigSpace ()
//NOTCHECKED      const ExtraConfigSpace& extraConfigSpace () const
//NOTCHECKED      virtual void setDimensionExtraConfigSpace (const size_type& dimension)
//      const Configuration_t& currentConfiguration () const
//      virtual bool currentConfiguration (ConfigurationIn_t configuration);
//NOTCHECKED      Configuration_t neutralConfiguration () const;
//      const vector_t& currentVelocity () const
//      void currentVelocity (vectorIn_t velocity)
//      const vector_t& currentAcceleration () const
//      void currentAcceleration (vectorIn_t acceleration)
//      const value_type& mass () const;
//      vector3_t positionCenterOfMass () const;
//      const ComJacobian_t& jacobianCenterOfMass () const;
//NOTCHECKED      void addGripper (const GripperPtr_t& gripper)
//NOTCHECKED      Grippers_t& grippers ()
//NOTCHECKED      const Grippers_t& grippers () const
//NOTCHECKED      const ObjectVector_t& obstacles (Request_t type) const;
//NOTCHECKED      virtual void addCollisionPairs ;
//NOTCHECKED      virtual void removeCollisionPairs;
//NOTCHECKED      const CollisionPairs_t& collisionPairs (Request_t type) const;
//NOTCHECKED      ObjectIterator objectIterator (Request_t type);
//NOTCHECKED      bool collisionTest () const;
//NOTCHECKED      void computeDistances ();
//NOTCHECKED      const DistanceResults_t& distanceResults () const ;
//NOTCHECKED      void controlComputation (const Computation_t& flag)
//NOTCHECKED      Computation_t computationFlag () const
//NOTCHECKED      virtual void computeForwardKinematics ();
//NOTCHECKED      virtual std::ostream& print (std::ostream& os) const;
// protected:
//      Device(const std::string& name);
//NOTCHECKED      void updateDistances ();
//NOTCHECKED      Device(const Device& device);
//      void resizeState ();
//NOTCHECKED      void resizeJacobians ();
// protected:
//NOTCHECKED      ModelPtr_t model_;
//NOTCHECKED      DataPtr_t data_;
//NOTCHECKED      std::string name_;
//NOTCHECKED      DistanceResults_t distances_;
//NOTCHECKED      Configuration_t currentConfiguration_;
//NOTCHECKED      vector_t currentVelocity_;
//NOTCHECKED      vector_t currentAcceleration_;
//NOTCHECKED      bool upToDate_;
//NOTCHECKED      Computation_t computationFlag_;
//NOTCHECKED      CollisionPairs_t collisionPairs_;
//NOTCHECKED      CollisionPairs_t distancePairs_;
//NOTCHECKED      Grippers_t grippers_;
//NOTCHECKED      ExtraConfigSpace extraConfigSpace_;
//NOTCHECKED      DeviceWkPtr_t weakPtr_;

/* --- UNIT TESTS ----------------------------------------------------------- */
/* --- UNIT TESTS ----------------------------------------------------------- */
/* --- UNIT TESTS ----------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (create)
{
  hpp::model::DevicePtr_t model = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  std::ofstream log ("./tdevice.log");
  log << *(model.get ()) << std::endl;
}

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (easyGetter)
{
  hpp::model::DevicePtr_t model = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  /* --- Check NQ NV */
  if(verbose) 
    {
      std::cout << "nq = " << model->configSize() << " vs " << pinocchio->configSize() << std::endl;
      std::cout << "nq = " << model->numberDof()  << " vs " << pinocchio->numberDof()  << std::endl;
    }

  BOOST_CHECK ( model->configSize()==pinocchio->configSize() );
  BOOST_CHECK ( model->numberDof() ==pinocchio->numberDof() );

  /* --- Check configuration */
  {
    BOOST_CHECK ( model    ->currentConfiguration().size()==model    ->configSize() );
    BOOST_CHECK ( pinocchio->currentConfiguration().size()==pinocchio->configSize() );
    Eigen::VectorXd q = Eigen::VectorXd::Random( model->configSize() );
    const Eigen::VectorXd qcopy = q;
    model->currentConfiguration(q);
    BOOST_CHECK( model->currentConfiguration() == qcopy );
    q = qcopy;
    pinocchio->currentConfiguration(q);
    BOOST_CHECK( pinocchio->currentConfiguration() == qcopy );
  }

  /* --- Check vel acc */
  {
    BOOST_CHECK ( model    ->currentVelocity().size()==model    ->numberDof() );
    BOOST_CHECK ( pinocchio->currentVelocity().size()==pinocchio->numberDof() );
    BOOST_CHECK ( model    ->currentAcceleration().size()==model    ->numberDof() );
    BOOST_CHECK ( pinocchio->currentAcceleration().size()==pinocchio->numberDof() );

    Eigen::VectorXd q = Eigen::VectorXd::Random( model->numberDof() );
    const Eigen::VectorXd qcopy = q;
    model->currentVelocity(q);
    BOOST_CHECK( model->currentVelocity() == qcopy );
    q = qcopy;
    pinocchio->currentVelocity(q);
    BOOST_CHECK( pinocchio->currentVelocity() == qcopy );
    q = qcopy;
     model->currentAcceleration(q);
    BOOST_CHECK( model->currentAcceleration() == qcopy );
    q = qcopy;
    pinocchio->currentAcceleration(q);
    BOOST_CHECK( pinocchio->currentAcceleration() == qcopy );
  }
}

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (compute)
{
  hpp::model::DevicePtr_t model = hppModel();
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
      std::cout << "com_model     = " << model    ->positionCenterOfMass() << std::endl;
      std::cout << "com_pinocchio = " << pinocchio->positionCenterOfMass() << std::endl;

      std::cout << model    ->jacobianCenterOfMass().leftCols<9>() << std::endl;
      std::cout << pinocchio->jacobianCenterOfMass().leftCols<9>() << std::endl;
    }

  BOOST_CHECK( model->mass() == pinocchio->mass() );

  BOOST_CHECK( model->positionCenterOfMass().isApprox
               (pinocchio->positionCenterOfMass()) );

  Eigen::MatrixXd Jmodel     = model    ->jacobianCenterOfMass();
  Eigen::MatrixXd Jpinocchio = pinocchio->jacobianCenterOfMass();
  Eigen::MatrixXd oRb = Jpinocchio.leftCols<3>();
  
  BOOST_CHECK( (Jmodel    *p2m::Xq(oRb)).isApprox(Jpinocchio) );
  BOOST_CHECK( (Jpinocchio*m2p::Xq(oRb)).isApprox(Jmodel) );
}


BOOST_AUTO_TEST_CASE(jointAccess)
{
  static bool verbose = true;
  hpp::model::DevicePtr_t model = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  BOOST_CHECK (pinocchio->rootJoint()->rankInConfiguration() == 0);

  if(verbose)
    std::cout << pinocchio->getJointAtConfigRank(0)->name() << " -- "
              <<  pinocchio->rootJoint()->name() <<std::endl;
  BOOST_CHECK (pinocchio->getJointAtConfigRank(0)->name() == pinocchio->rootJoint()->name());

  BOOST_CHECK (pinocchio->getJointAtVelocityRank(0)->name() == pinocchio->rootJoint()->name());

  for (int i=1;i<pinocchio->model()->njoint;++i)
    {
      BOOST_CHECK( pinocchio->getJointAtConfigRank(pinocchio->model()->joints[i].idx_q())->name()
                   == pinocchio->model()->names[i] );
      BOOST_CHECK( pinocchio->getJointAtVelocityRank(pinocchio->model()->joints[i].idx_v())->name()
                   == pinocchio->model()->names[i] );
    }
  
  hpp::model::JointPtr_t jm = model->getJointVector()[5];
  BOOST_CHECK( pinocchio->getJointByName (jm->name())->name() == jm->name() );

  // Compare the joint vector
  if (verbose)
    std::cout << "\n\nComparing joint vectors\n\n";

  hpp::model::size_type idM = 0;
  hpp::model    ::JointVector_t jvm = model    ->getJointVector();
  hpp::pinocchio::JointVector_t jvp = pinocchio->getJointVector();
  for (int idP=0;idP<jvp.size();++idP)
  {
    // Skip anchor joints for hpp-model
    while(idM < jvm.size() &&
        dynamic_cast<hpp::model::JointAnchor*>(jvm[idM]) != NULL) {
      idM++;
    }
    if(verbose)
      std::cout
        << jvp[idP]->name() << " -- "
        << jvm[idM]->name() << std::endl;
    se3::JointIndex jidP = jvp[idP]->index();
    BOOST_CHECK(jidP == idP + 1);
    if (verbose)
      std::cout << pinocchio->model()->joints[jidP].shortname() << std::endl;

    if (pinocchio->model()->joints[jidP].shortname() == "JointModelFreeFlyer") {
      // The root joint have different names in model and pinocchio
      if (idP == 0) { idM += 2; continue; }
      // Joint is a freeflyer
      std::string nameP = jvp[idP]->name();
      BOOST_CHECK( jvm[idM]->name().compare(0, nameP.length(), nameP) == 0);
      idM++;
      BOOST_CHECK( jvm[idM]->name().compare(0, nameP.length(), nameP) == 0);
      idM++;
    } else {
      // Joint is a neither an anchor nor a freeflyer
      BOOST_CHECK(jvp[idP]->name() == jvm[idM]->name());
      idM++;
    }
  }
  // Check that all the remainings joint of hpp-model are anchor joints.
  while(idM != jvm.size()) {
    BOOST_CHECK(dynamic_cast<hpp::model::JointAnchor*>(jvm[idM]) != NULL);
    idM++;
  }
}
