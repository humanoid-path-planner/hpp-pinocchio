// Copyright (c) 2017, CNRS
// Authors: Florent Lamiraux
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

#include <hpp/pinocchio/liegroup-element.hh>
#include <hpp/pinocchio/liegroup-space.hh>

namespace hpp {
  namespace pinocchio {

    LiegroupSpacePtr_t LiegroupSpace::Rn (const size_type& n)
    {
      LiegroupSpacePtr_t  S (new LiegroupSpace ());
      S->liegroupTypes_.push_back
        (se3::VectorSpaceOperation <Eigen::Dynamic> ((int)n));
      S->nq_ = S->nv_ = n;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      std::ostringstream oss; oss << "R^" << n;
      S->name_ = oss.str ();
      return S;
    }

    /// Return \f$\mathbf{R}\f$ as a Lie group
    LiegroupSpacePtr_t LiegroupSpace::R1 ()
    {
      LiegroupSpacePtr_t  S (new LiegroupSpace ());
      S->liegroupTypes_.push_back (se3::VectorSpaceOperation <1> ());
      S->nq_ = S->nv_ = 1;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      S->name_ = "R";
      return S;
    }

    /// Return \f$\mathbf{R}^2\f$ as a Lie group
    LiegroupSpacePtr_t LiegroupSpace::R2 ()
    {
      LiegroupSpacePtr_t  S (new LiegroupSpace ());
      S->liegroupTypes_.push_back (se3::VectorSpaceOperation <2> ());
      S->nq_ = S->nv_ = 2;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      S->name_ = "R^2";
      return S;
    }

      /// Return \f$\mathbf{R}^3\f$ as a Lie group
    LiegroupSpacePtr_t LiegroupSpace::R3 ()
    {
      LiegroupSpacePtr_t  S (new LiegroupSpace ());
      S->liegroupTypes_.push_back (se3::VectorSpaceOperation <3> ());
      S->nq_ = S->nv_ = 3;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      S->name_ = "R^3";
      return S;
    }

      /// Return \f$SE(2)\f$
    LiegroupSpacePtr_t LiegroupSpace::SE2 ()
    {
      LiegroupSpacePtr_t  S (new LiegroupSpace ());
      S->liegroupTypes_.push_back (se3::SpecialEuclideanOperation<2> ());
      S->nq_ = se3::SpecialEuclideanOperation<2>::NQ;
      S->nv_ = se3::SpecialEuclideanOperation<2>::NV;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      S->neutral_ [2] = 1;
      S->name_ = "SE(2)";
      return S;
    }

    /// Return \f$SE(3)\f$
    LiegroupSpacePtr_t LiegroupSpace::SE3 ()
    {
      LiegroupSpacePtr_t  S (new LiegroupSpace ());
      S->liegroupTypes_.push_back (se3::SpecialEuclideanOperation<3> ());
      S->nq_ = se3::SpecialEuclideanOperation<3>::NQ;
      S->nv_ = se3::SpecialEuclideanOperation<3>::NV;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      S->neutral_ [6] = 1;
      S->name_ = "SE(3)";
      return S;
    }

    /// Return \f$SO(2)\f$
    LiegroupSpacePtr_t LiegroupSpace::SO2 ()
    {
      LiegroupSpacePtr_t  S (new LiegroupSpace ());
      S->liegroupTypes_.push_back (se3::SpecialOrthogonalOperation<2> ());
      S->nq_ = se3::SpecialOrthogonalOperation<2>::NQ;
      S->nv_ = se3::SpecialOrthogonalOperation<2>::NV;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      S->neutral_ [0] = 1;
      S->name_ = "SO(2)";
      return S;
    }

    /// Return \f$SO(3)\f$
    LiegroupSpacePtr_t LiegroupSpace::SO3 ()
    {
      LiegroupSpacePtr_t  S (new LiegroupSpace ());
      S->liegroupTypes_.push_back (se3::SpecialOrthogonalOperation<3> ());
      S->nq_ = se3::SpecialOrthogonalOperation<3>::NQ;
      S->nv_ = se3::SpecialOrthogonalOperation<3>::NV;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      S->neutral_ [3] = 1;
      S->name_ = "SO(3)";
      return S;
      }

    /// Return empty Lie group
    LiegroupSpacePtr_t LiegroupSpace::empty ()
    {
      LiegroupSpacePtr_t  S (new LiegroupSpace ());
      S->nq_ = S->nv_ = 0;
      S->neutral_.resize (S->nq_);
      return S;
    }

    vector_t LiegroupSpace::neutral () const
    {
      return neutral_;
    }

    LiegroupSpacePtr_t operator*
    (const LiegroupSpacePtr_t& sp1, const LiegroupSpacePtr_t& sp2)
    {
      LiegroupSpacePtr_t res (LiegroupSpace::create (sp1));
      res->liegroupTypes_.insert (res->liegroupTypes_.end (),
                                  sp2->liegroupTypes_.begin (),
                                  sp2->liegroupTypes_.end ());
      res->nq_ = sp1->nq_ + sp2->nq_;
      res->nv_ = sp1->nv_ + sp2->nv_;
      res->neutral_.resize (res->nq_);
      res->neutral_.head (sp1->nq ()) = sp1->neutral ();
      res->neutral_.tail (sp2->nq ()) = sp2->neutral ();
      res->name_ = sp1->name ();
      if (sp1->name () != "" && sp2->name () != "") res->name_ += "*";
      res->name_ += sp2->name ();
      return res;
    }

  } // namespace pinocchio
} // namespace hpp
