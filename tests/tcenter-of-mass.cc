// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-pinocchio.
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
// hpp-pinocchio. If not, see <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE tcenterOfMass

#include <boost/test/unit_test.hpp>

#include <hpp/model/device.hh>
#include <hpp/model/urdf/util.hh>
#include <hpp/model/center-of-mass-computation.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
#include "../tests/utils.hh"

static bool verbose = false;

/* --- UNIT TESTS ----------------------------------------------------------- */
/* --- UNIT TESTS ----------------------------------------------------------- */
/* --- UNIT TESTS ----------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (CenterOfMassComputation)
{
  namespace model = hpp::model;
  namespace pinoc = hpp::pinocchio;

  model::DevicePtr_t rm = hppModel();
  pinoc::DevicePtr_t rp = hppPinocchio(true);

  model::CenterOfMassComputationPtr_t comM = model::CenterOfMassComputation::create(rm);
  pinoc::CenterOfMassComputationPtr_t comP = pinoc::CenterOfMassComputation::create(rp);

  comM->add(rm->rootJoint());
  comP->add(rp->rootJoint());

  comM->computeMass();
  comP->computeMass();

  Eigen::VectorXd q = Eigen::VectorXd::Random( rm->configSize() );
  q[3] += 1.0;
  q.segment<4>(3) /= q.segment<4>(3).norm();

  rm->currentConfiguration(q);
  rm->controlComputation (model::Device::ALL);
  rm->computeForwardKinematics();

  rp->currentConfiguration(m2p::q(q));
  rp->controlComputation (pinoc::Device::ALL);
  rp->computeForwardKinematics();

  comM->compute(model::Device::COM);
  comP->compute(pinoc::Device::COM);
  BOOST_CHECK(comM->com().isApprox(comP->com()));

  comM->compute(model::Device::ALL);
  comP->compute(pinoc::Device::ALL);
  BOOST_CHECK(comM->com().isApprox(comP->com()));
  BOOST_CHECK(comM->jacobian().isApprox(comP->jacobian() * m2p::Xq(rp->rootJoint()->currentTransformation())));
}

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (easyGetter)
{
  hpp::model::DevicePtr_t model = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  model->setDimensionExtraConfigSpace(2);
  pinocchio->setDimensionExtraConfigSpace(2);

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

    std::cout << pinocchio->neutralConfiguration().transpose() << std::endl;
    std::cout << model->neutralConfiguration().transpose() << std::endl;
    // This does not work.
    // BOOST_CHECK( pinocchio->neutralConfiguration().isApprox( m2p::q(model->neutralConfiguration()) ) );

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

  // Skip root joint because the name is not the same.
  for (int i=2;i<pinocchio->model()->njoint;++i)
    {
      const std::string& name = pinocchio->model()->names[i];
      hpp::model    ::JointPtr_t jm = model    ->getJointByName (name);
      hpp::pinocchio::JointPtr_t jp = pinocchio->getJointByName (name);

      hpp::model    ::Transform3f tfm = jm->currentTransformation();
      hpp::pinocchio::Transform3f tfp = jp->currentTransformation();

      // The center of the joint frames should be the same.
      BOOST_CHECK( tfm.getTranslation().isApprox(tfp.translation()) );
      // The rotations may be permuted because HPP only has a rotation around X.
      // To be checked using urdf link position (see hpp-constraints/pmdiff/tvalue.cc
    }
  
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

void checkJointBound(const hpp::model::JointPtr_t& jm, const hpp::pinocchio::JointPtr_t& jp)
{
  BOOST_CHECK(jp->configSize() == jm->configSize());
  for (int rk = 0; rk < jm->configSize(); ++rk) {
    BOOST_CHECK(jp->isBounded(rk) == jm->isBounded(rk));
    if (jm->isBounded(rk)) {
      BOOST_CHECK(jp->lowerBound(rk) == jm->lowerBound(rk));
      BOOST_CHECK(jp->upperBound(rk) == jm->upperBound(rk));
    }
  }
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
      // Check joint bound
      checkJointBound (jvm[idM], jvp[idP]);
      idM++;
    }
  }
  // Check that all the remainings joint of hpp-model are anchor joints.
  while(idM != jvm.size()) {
    BOOST_CHECK(dynamic_cast<hpp::model::JointAnchor*>(jvm[idM]) != NULL);
    idM++;
  }
}
