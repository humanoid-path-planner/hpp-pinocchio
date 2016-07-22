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

#include "hpp/pinocchio/joint.hh"
#include "hpp/pinocchio/device.hh"

namespace hpp {
  namespace pinocchio {
    struct CenterOfMassComputation::JointSubtree_t {
      JointPtr_t j_;
      value_type mass_;
      JointSubtree_t (const JointPtr_t& joint) : j_ (joint) {}
    };

    CenterOfMassComputationPtr_t CenterOfMassComputation::create (
        const DevicePtr_t& d)
    {
      return CenterOfMassComputationPtr_t (new CenterOfMassComputation (d));
    }

    void CenterOfMassComputation::computeMass ()
    {
      mass_ = 0;
      for (JointSubtrees_t::iterator _subtree = jointSubtrees_.begin ();
          _subtree != jointSubtrees_.end (); ++_subtree) {
        // TODO: Compute the mass of the subtree
        // Its root joint is (JointPtr_t): _subtree->j_
        //
        // _subtree->mass_ = ???
        mass_ += _subtree->mass_;
      }
      assert (mass_ > 0);
    }

    void CenterOfMassComputation::compute (const Device::Computation_t& flag)
    {
      assert (mass_ > 0);
      if (flag & Device::COM) {
        com_.setZero();
        for (JointSubtrees_t::iterator _subtree = jointSubtrees_.begin ();
            _subtree != jointSubtrees_.end (); ++_subtree) {
          vector3_t com;
          assert(false && "Unimplemented");
          // TODO: Compute com of subtree
          // Its root joint is (JointPtr_t): _subtree->j_
          //
          // com = ???
          com_ += _subtree->mass_ * com;
        }
        com_ /= mass_;
      }
      if (flag & Device::JACOBIAN) {
        jacobianCom_.setZero ();
        for (JointSubtrees_t::iterator _subtree = jointSubtrees_.begin ();
            _subtree != jointSubtrees_.end (); ++_subtree) {
          ComJacobian_t Jcom;
          assert(false && "Unimplemented");
          // TODO: Compute jacobian of subtree
          // Its root joint is (JointPtr_t): _subtree->j_
          //
          // Jcom = ???
          jacobianCom_ += _subtree->mass_ * Jcom;
        }
        jacobianCom_ /= mass_;
      }
    }

    CenterOfMassComputation::CenterOfMassComputation (const DevicePtr_t& d) :
      jointSubtrees_ (), mass_ (-1), jacobianCom_ (3, d->numberDof ())
    {}

    void CenterOfMassComputation::add (const JointPtr_t& j)
    {
      se3::JointIndex jid = j->index();
      const se3::Model& m = *j->robot()->model();
      for (JointSubtrees_t::iterator _subtree = jointSubtrees_.begin ();
          _subtree != jointSubtrees_.end (); ++_subtree) {
        se3::JointIndex sbId = _subtree->j_->index();
        // Check that jid is not in the subtree
        for (se3::JointIndex id = jid; id != 0; id = m.parents[id])
          if (id == sbId) {
            // We are doing something stupid. Should we throw an error
            // or just return silently ?
            throw std::invalid_argument("This joint is already in a subtree");
            // return;
          }
      }

      jointSubtrees_.push_back(JointSubtree_t(j));
    }

    CenterOfMassComputation::~CenterOfMassComputation ()
    {}
  }  //  namespace pinocchio
}  //  namespace hpp
