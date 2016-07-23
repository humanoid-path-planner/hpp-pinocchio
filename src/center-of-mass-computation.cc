// Copyright (c) 2016, LAAS-CNRS
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

#include "hpp/pinocchio/center-of-mass-computation.hh"

#include <algorithm>
#include <queue>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include "hpp/pinocchio/joint.hh"
#include "hpp/pinocchio/device.hh"

#include "center-of-mass-computation/algorithm.cc"

namespace hpp {
  namespace pinocchio {
    CenterOfMassComputationPtr_t CenterOfMassComputation::create (
        const DevicePtr_t& d)
    {
      return CenterOfMassComputationPtr_t (new CenterOfMassComputation (d));
    }

    void CenterOfMassComputation::compute (const Device::Computation_t& flag)
    {
      assert (mass_ > 0);
      const se3::Model& model = *robot_->model();
      se3::Data& data = *robot_->data();
      bool computeCOM = (flag & Device::COM);
      bool computeJac = (flag & Device::JACOBIAN);
      assert(computeCOM && "This does nothing");
      assert (!(computeJac && !computeCOM)); // JACOBIAN => COM
      if (computeJac) {
        com_.setZero();
        jacobianCom_.setZero ();
        for (std::size_t i = 0; i < joints_.size(); ++i) {
          se3::subtreeJacobianCenterOfMass(model, data, joints_[i]);
          se3::JointIndex j = joints_[i].root;
          com_ += data.mass[j] * data.com[j];
          jacobianCom_ += data.mass[j] * data.Jcom;
        }
        com_ /= mass_;
        jacobianCom_ /= mass_;
      } else if (computeCOM) {
        com_.setZero();
        se3::centerOfMass(model, data,
            robot_->currentConfiguration(), true, false);
        for (std::size_t i = 0; i < joints_.size(); ++i) {
          se3::JointIndex j = joints_[i].root;
          com_ += data.mass[j] * data.oMi[j].act(data.com[j]);
        }
        com_ /= mass_;
      }
    }

    CenterOfMassComputation::CenterOfMassComputation (const DevicePtr_t& d) :
      robot_(d), joints_ (), mass_ (0), jacobianCom_ (3, d->numberDof ())
    {}

    void CenterOfMassComputation::add (const JointPtr_t& j)
    {
      se3::JointIndex jid = j->index();
      const se3::Model& m = *robot_->model();
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        se3::JointIndex sbId = joints_[i].root;
        // Check that jid is not in the subtree
        for (se3::JointIndex id = jid; id != 0; id = m.parents[id])
          if (id == sbId) {
            // We are doing something stupid. Should we throw an error
            // or just return silently ?
            throw std::invalid_argument("This joint is already in a subtree");
            // return;
          }
      }

      joints_.push_back(se3::SubtreeModel());
      joints_.back().root = jid;
      se3::subtreeIndexes(m, joints_.back());
    }

    void CenterOfMassComputation::computeMass ()
    {
      const se3::Model& model = *robot_->model();
      se3::Data& data = *robot_->data();
      se3::centerOfMass(model, data,
          robot_->currentConfiguration(), true, false);
      mass_ = 0;
      for (std::size_t i = 0; i < joints_.size(); ++i)
        mass_ += data.mass[i];
    }

    CenterOfMassComputation::~CenterOfMassComputation ()
    {}
  }  //  namespace pinocchio
}  //  namespace hpp
