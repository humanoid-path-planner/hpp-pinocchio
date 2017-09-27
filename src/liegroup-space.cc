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
#include "../src/comparison.hh"

namespace hpp {
  namespace pinocchio {

    LiegroupSpacePtr_t LiegroupSpace::Rn (const size_type& n)
    {
      LiegroupSpace* ptr (new LiegroupSpace ());
      LiegroupSpacePtr_t  S (ptr);
      ptr->init (S);
      S->liegroupTypes_.push_back
        (liegroup::VectorSpaceOperation <Eigen::Dynamic, false> ((int)n));
      S->nq_ = S->nv_ = n;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      return S;
    }

    /// Return \f$\mathbf{R}\f$ as a Lie group
    LiegroupSpacePtr_t LiegroupSpace::R1 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace ());
      LiegroupSpacePtr_t  S (ptr);
      ptr->init (S);
      S->liegroupTypes_.push_back (liegroup::VectorSpaceOperation <1, false> ());
      S->nq_ = S->nv_ = 1;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      return S;
    }

    /// Return \f$\mathbf{R}^2\f$ as a Lie group
    LiegroupSpacePtr_t LiegroupSpace::R2 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace ());
      LiegroupSpacePtr_t  S (ptr);
      ptr->init (S);
      S->liegroupTypes_.push_back (liegroup::VectorSpaceOperation <2, false> ());
      S->nq_ = S->nv_ = 2;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      return S;
    }

      /// Return \f$\mathbf{R}^3\f$ as a Lie group
    LiegroupSpacePtr_t LiegroupSpace::R3 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace ());
      LiegroupSpacePtr_t  S (ptr);
      ptr->init (S);
      S->liegroupTypes_.push_back (liegroup::VectorSpaceOperation <3, false> ());
      S->nq_ = S->nv_ = 3;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      return S;
    }

    /// Return \f$SO(2)\f$
    LiegroupSpacePtr_t LiegroupSpace::SO2 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace ());
      LiegroupSpacePtr_t  S (ptr);
      ptr->init (S);
      S->liegroupTypes_.push_back (liegroup::SpecialOrthogonalOperation<2> ());
      S->nq_ = liegroup::SpecialOrthogonalOperation<2>::NQ;
      S->nv_ = liegroup::SpecialOrthogonalOperation<2>::NV;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      S->neutral_ [0] = 1;
      return S;
    }

    /// Return \f$SO(3)\f$
    LiegroupSpacePtr_t LiegroupSpace::SO3 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace ());
      LiegroupSpacePtr_t  S (ptr);
      ptr->init (S);
      S->liegroupTypes_.push_back (liegroup::SpecialOrthogonalOperation<3> ());
      S->nq_ = liegroup::SpecialOrthogonalOperation<3>::NQ;
      S->nv_ = liegroup::SpecialOrthogonalOperation<3>::NV;
      S->neutral_.resize (S->nq_); S->neutral_.setZero ();
      S->neutral_ [3] = 1;
      return S;
      }

    /// Return empty Lie group
    LiegroupSpacePtr_t LiegroupSpace::empty ()
    {
      LiegroupSpace* ptr (new LiegroupSpace ());
      LiegroupSpacePtr_t  S (ptr);
      ptr->init (S);
      S->nq_ = S->nv_ = 0;
      S->neutral_.resize (S->nq_);
      return S;
    }

    LiegroupElement LiegroupSpace::neutral () const
    {
      return LiegroupElement (neutral_, weak_.lock ());
    }

    struct NameVisitor : public boost::static_visitor <>
    {
      template <typename LgT1> void operator () (const LgT1& lg1)
      {
        name = lg1.name ();
      }
      std::string name;
    }; // struct NameVisitor

    std::string LiegroupSpace::name () const
    {
      std::ostringstream oss;
      std::size_t size (liegroupTypes_.size ());
      if (size == 0) {
        return std::string ("");
      }
      for (std::size_t i = 0; i < liegroupTypes_.size (); ++i) {
        NameVisitor v;
        boost::apply_visitor (v, liegroupTypes_ [i]);
        oss << v.name;
        if (i < liegroupTypes_.size () - 1) {
          oss << "*";
        }
      }
      return oss.str ();
    }

    bool LiegroupSpace::operator== (const LiegroupSpace& other) const
    {
      if (liegroupTypes_.size () != other.liegroupTypes ().size ())
        return false;
      LiegroupTypes::const_iterator it1 (liegroupTypes_.begin ());
      LiegroupTypes::const_iterator it2 (other.liegroupTypes ().begin ());
      while (it1 != liegroupTypes_.end ()){
        liegroup::level1::IsEqualVisitor v (*it2);
        boost::apply_visitor (v, *it1);
        if (!v.result) return false;
        ++it1; ++it2;
      }
      return true;
      //return (liegroupTypes_ == other.liegroupTypes ());
    }

    bool LiegroupSpace::operator!= (const LiegroupSpace& other) const
    {
      return !(operator== (other));
    }

    void LiegroupSpace::init (const LiegroupSpaceWkPtr_t weak)
    {
      weak_ = weak;
    }

    LiegroupSpacePtr_t operator*
    (const LiegroupSpacePtr_t& sp1, const LiegroupSpacePtr_t& sp2)
    {
      LiegroupSpacePtr_t res (LiegroupSpace::createCopy (sp1));
      res->liegroupTypes_.insert (res->liegroupTypes_.end (),
                                  sp2->liegroupTypes_.begin (),
                                  sp2->liegroupTypes_.end ());
      res->nq_ = sp1->nq_ + sp2->nq_;
      res->nv_ = sp1->nv_ + sp2->nv_;
      res->neutral_.resize (res->nq_);
      res->neutral_.head (sp1->nq ()) = sp1->neutral ().vector ();
      res->neutral_.tail (sp2->nq ()) = sp2->neutral ().vector ();
      return res;
    }

  } // namespace pinocchio
} // namespace hpp
