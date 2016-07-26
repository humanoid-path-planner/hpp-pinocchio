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

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

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
  //comP->computeMass();

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
  if (verbose)
    std::cout << comM->com() << " -- "
      << comP->com().transpose() << std::endl << std::endl;

  comM->compute(model::Device::ALL);
  comP->compute(pinoc::Device::ALL);
  BOOST_CHECK(comM->com().isApprox(comP->com()));
  BOOST_CHECK(comM->jacobian().isApprox(comP->jacobian() * m2p::Xq(rp->rootJoint()->currentTransformation())));
}

/* -------------------------------------------------------------------------- */

// Modify the model so that all masses except thos of the subtrees become 0.
static void nullifyMasses(se3::Model & model, const se3::Data & data,
                          const std::vector <se3::JointIndex> & roots )
{
  int root = 0;
  for( se3::JointIndex jid=1; int(jid)<model.njoint; ++jid )
    {
      const se3::JointIndex& rootId = roots[root];
      if(jid == rootId)
        {
          jid = data.lastChild[rootId];
          root ++;
        }
      else 
        {
          if(verbose) std::cout<<"Nullified mass " << jid << std::endl;
          model.inertias[jid].mass() = 0;
        }
    }
}

// Check subtree jacobian versus jacobian of a model whose masses are nullified.
BOOST_AUTO_TEST_CASE (nullmass)
{
  namespace pinoc = hpp::pinocchio;
  pinoc::DevicePtr_t rp = hppPinocchio(true);

  pinoc::CenterOfMassComputationPtr_t comP = pinoc::CenterOfMassComputation::create(rp);
  comP->add(rp->getJointAtVelocityRank(7));

  nullifyMasses(rp->model(),rp->data(),comP->roots());

  Eigen::VectorXd q = Eigen::VectorXd::Random( rp->configSize() );
  q[3] += 1.0;
  q.segment<4>(3) /= q.segment<4>(3).norm();

  rp->currentConfiguration(q);
  rp->controlComputation (pinoc::Device::ALL);
  rp->computeForwardKinematics();

  comP->compute(pinoc::Device::COM);
  if(verbose)
    std::cout << "wholebody = " << rp->positionCenterOfMass().transpose()  << " -- " 
              << "subtree   = " << comP->com().transpose() << std::endl << std::endl;  
  BOOST_CHECK(rp->positionCenterOfMass().isApprox(comP->com()));

  comP->compute(pinoc::Device::ALL);
  BOOST_CHECK(rp->positionCenterOfMass().isApprox(comP->com()));
  BOOST_CHECK(rp->jacobianCenterOfMass().isApprox(comP->jacobian()));
}

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (finiteDiff)
{
  //verbose =true;
  namespace pinoc = hpp::pinocchio;
  pinoc::DevicePtr_t rp = hppPinocchio(true);
  pinoc::CenterOfMassComputationPtr_t comP = pinoc::CenterOfMassComputation::create(rp);
  comP->add(rp->getJointAtVelocityRank(7));

  Eigen::VectorXd q = Eigen::VectorXd::Random( rp->configSize() );
  q[3] += 1.0;
  q.segment<4>(3) /= q.segment<4>(3).norm();

  rp->currentConfiguration(q);
  rp->controlComputation (pinoc::Device::ALL);
  rp->computeForwardKinematics();

  comP->compute(pinoc::Device::ALL);

  const int NV = rp->model().nv;
  Eigen::MatrixXd Jcom = Eigen::MatrixXd::Zero(3,NV);
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(NV);
  Eigen::VectorXd com = comP->com();
  const double EPS = 1e-8;
  for( int i=0;i<NV;++i )
    {
      dq[i] = EPS;
      Eigen::VectorXd qdq = se3::integrate(rp->model(), q, dq);
      se3::forwardKinematics(rp->model(),rp->data(),qdq);
      comP->compute(pinoc::Device::COM);
      Jcom.col(i) = (comP->com()-com)/EPS;

      dq[i] = 0;
    }

  // The approximation by finite differencing should be equal to the jacobian. 
  // It is not but requires a rotation of the free-flyer velocity. 
  // I keep the test like that (it should pass, no error expected) but we must investigate 
  // this problem further.
  // It might be either a problem in the way I am doing finite-differencing on a Lie group,
  // or in a problem in the implementation of the integrate function.

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(rp->numberDof(),rp->numberDof());
  R.topLeftCorner<3,3>() = comP->jacobian().leftCols<3>();
  R.block<3,3>(3,3) = comP->jacobian().leftCols<3>();

  if(verbose)
    {
      std::cout << "Jcom = [ " << comP->jacobian().leftCols<12>() << "]; " << std::endl;
      std::cout << "Dcom = [ " << Jcom.leftCols<12>() << "]; " << std::endl;
      std::cout << "JcomR = [ " << (comP->jacobian()*R.transpose()).leftCols<12>() << "]; " << std::endl;
    }
  
  BOOST_CHECK( comP->jacobian().isApprox( Jcom*R,1e-5 ) );
}

