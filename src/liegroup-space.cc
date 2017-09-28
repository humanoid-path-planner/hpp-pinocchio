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
#include "../src/size-visitor.hh"

namespace hpp {
  namespace pinocchio {

    LiegroupSpacePtr_t LiegroupSpace::Rn (const size_type& n)
    {
      LiegroupSpace* ptr (new LiegroupSpace (n));
      LiegroupSpacePtr_t  shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    /// Return \f$\mathbf{R}\f$ as a Lie group
    LiegroupSpacePtr_t LiegroupSpace::R1 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace
                          (liegroup::VectorSpaceOperation <1, false> ()));
      LiegroupSpacePtr_t  shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    /// Return \f$\mathbf{R}^2\f$ as a Lie group
    LiegroupSpacePtr_t LiegroupSpace::R2 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace
                          (liegroup::VectorSpaceOperation <2, false> ()));
      LiegroupSpacePtr_t  shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    /// Return \f$\mathbf{R}^3\f$ as a Lie group
    LiegroupSpacePtr_t LiegroupSpace::R3 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace
                          (liegroup::VectorSpaceOperation <3, false> ()));
      LiegroupSpacePtr_t  shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    /// Return \f$R^2\times SO(2)\f$
    LiegroupSpacePtr_t LiegroupSpace::R2xSO2 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace
                          (liegroup::CartesianProductOperation<
                           liegroup::VectorSpaceOperation<2, false>,
                           liegroup::SpecialOrthogonalOperation<2> > ()));
      LiegroupSpacePtr_t  shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    /// Return \f$R^3\times SO(3)\f$
    LiegroupSpacePtr_t LiegroupSpace::R3xSO3 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace
                          (liegroup::CartesianProductOperation<
                           liegroup::VectorSpaceOperation<3, false>,
                           liegroup::SpecialOrthogonalOperation<3> > ()));
      LiegroupSpacePtr_t  shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    /// Return empty Lie group
    LiegroupSpacePtr_t LiegroupSpace::empty ()
    {
      LiegroupSpace* ptr (new LiegroupSpace ());
      LiegroupSpacePtr_t  shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
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

    // Constructors
    LiegroupSpace::LiegroupSpace (const size_type& size) :
      nq_ (0), nv_ (0), neutral_ (), weak_ ()
    {
      liegroupTypes_.push_back
        (liegroup::VectorSpaceOperation <Eigen::Dynamic, false> ((int) size));
      computeSize ();
      neutral_.resize (nq_);
      neutral_.setZero ();
    }

    LiegroupSpace::LiegroupSpace (const LiegroupSpace& other) :
      liegroupTypes_ (other.liegroupTypes_), nq_ (other.nq_), nv_ (other.nv_),
      neutral_ (other.neutral_), weak_ ()
    {
    }

    LiegroupSpace::LiegroupSpace (const LiegroupType& type) :
      liegroupTypes_ (), neutral_ (), weak_ ()
    {
      liegroupTypes_.push_back (type);
      computeSize ();
      computeNeutral ();
    }

    LiegroupSpace::LiegroupSpace () : liegroupTypes_ (), neutral_ (), weak_ ()
    {
      computeSize ();
      computeNeutral ();
    }

    void LiegroupSpace::computeSize ()
    {
      nq_ = nv_ = 0;
      for (LiegroupTypes::const_iterator it = liegroupTypes_.begin ();
           it != liegroupTypes_.end (); ++it) {
        liegroupType::SizeVisitor v;
        boost::apply_visitor (v, *it);
        nq_ += v.nq;
        nv_ += v.nv;
      }
    }

    void LiegroupSpace::computeNeutral ()
    {
      neutral_.resize (nq_);
      size_type start = 0;
      for (LiegroupTypes::const_iterator it = liegroupTypes_.begin ();
           it != liegroupTypes_.end (); ++it) {
        liegroupType::NeutralVisitor v;
        boost::apply_visitor (v, *it);
        neutral_.segment (start, v.neutral.size ()) = v.neutral;
        start += v.neutral.size ();
      }
      assert (start == nq_);
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
