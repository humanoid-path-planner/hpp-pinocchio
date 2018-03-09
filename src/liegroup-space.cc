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
#include "../src/jintegrate-visitor.hh"
#include "../src/jdifference-visitor.hh"

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

    /// Return \f$SE(2)\f$
    LiegroupSpacePtr_t LiegroupSpace::SE2 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace
                          (se3::SpecialEuclideanOperation <2>()));
      LiegroupSpacePtr_t  shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    /// Return \f$SE(3)\f$
    LiegroupSpacePtr_t LiegroupSpace::SE3 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace
                          (se3::SpecialEuclideanOperation <3>()));
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

    LiegroupElement LiegroupSpace::exp (vectorIn_t v) const
    {
      return neutral () + v;
    }

    void LiegroupSpace::Jintegrate (vectorIn_t v, matrixOut_t J) const
    {
      assert (v.size() == nv());
      assert (J.rows() == nv());
      size_type row = 0;
      liegroupType::JintegrateVisitor jiv (v, J, row);
      for (std::size_t i = 0; i < liegroupTypes_.size (); ++i) {
        boost::apply_visitor (jiv, liegroupTypes_ [i]);
      }
      assert (row == nv());
    }

    void LiegroupSpace::Jdifference (vectorIn_t q0, vectorIn_t q1, matrixOut_t J0, matrixOut_t J1) const
    {
      assert (q0.size() == nq() && q1.size() == nq());

      liegroupType::JdifferenceVisitor jdv (q0,q1,J0,J1);
      for (std::size_t i = 0; i < liegroupTypes_.size (); ++i) {
        boost::apply_visitor (jdv, liegroupTypes_ [i]);
      }
      assert (jdv.iq_ == nq());
      assert (jdv.iv_ == nv());
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
    LiegroupSpace::LiegroupSpace () :
      nq_ (0), nv_ (0), nqs_ (), nvs_ (), neutral_ (), weak_ ()
    {}

    LiegroupSpace::LiegroupSpace (const size_type& size) :
      nq_ (0), nv_ (0), nqs_ (), nvs_ (), neutral_ (), weak_ ()
    {
      liegroupTypes_.push_back
        (liegroup::VectorSpaceOperation <Eigen::Dynamic, false> ((int) size));
      computeSize ();
      neutral_.resize (nq_);
      neutral_.setZero ();
    }

    LiegroupSpace::LiegroupSpace (const LiegroupSpace& other) :
      liegroupTypes_ (other.liegroupTypes_), nq_ (other.nq_), nv_ (other.nv_),
      nqs_ (other.nqs_), nvs_ (other.nvs_), neutral_ (other.neutral_), weak_ ()
    {
    }

    LiegroupSpace::LiegroupSpace (const LiegroupType& type) :
      liegroupTypes_ (), nqs_ (), nvs_ (), neutral_ (), weak_ ()
    {
      liegroupTypes_.push_back (type);
      computeSize ();
      computeNeutral ();
    }

    void LiegroupSpace::computeSize ()
    {
      nq_ = nv_ = 0;
      nqs_.clear();
      nvs_.clear();
      for (LiegroupTypes::const_iterator it = liegroupTypes_.begin ();
           it != liegroupTypes_.end (); ++it) {
        liegroupType::SizeVisitor v;
        boost::apply_visitor (v, *it);
        nq_ += v.nq;
        nv_ += v.nv;
        nqs_.push_back (v.nq);
        nvs_.push_back (v.nv);
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

    void LiegroupSpace::mergeVectorSpaces ()
    {
      if (liegroupTypes_.empty()) return;

      LiegroupTypes newLgT;

      liegroupType::IsVectorSpace ivs;
      liegroupType::SizeVisitor curSizes;
      size_type vsSize = 0;

      for (LiegroupTypes::const_iterator _cur = liegroupTypes_.begin();
          _cur != liegroupTypes_.end (); ++_cur) {
        boost::apply_visitor (ivs, *_cur);
        bool curIsVectorSpace = ivs.isVectorSpace;
        if (curIsVectorSpace) {
          boost::apply_visitor (curSizes, *_cur);
        }
        if (vsSize > 0 && curIsVectorSpace) {
          // Update previous liegroup type (which is a vector space).
          vsSize += curSizes.nq;
          assert (!newLgT.empty());
          switch (vsSize) {
            case  1: newLgT.back() = liegroup::VectorSpaceOperation<             1, false> (); break;
            case  2: newLgT.back() = liegroup::VectorSpaceOperation<             2, false> (); break;
            case  3: newLgT.back() = liegroup::VectorSpaceOperation<             3, false> (); break;
            default: newLgT.back() = liegroup::VectorSpaceOperation<Eigen::Dynamic, false> ((int)vsSize); break;
          }
        } else {
          // No merge to do.
          vsSize = (curIsVectorSpace ? curSizes.nq : 0);
          newLgT.push_back (*_cur);
        }
      }
      liegroupTypes_ = newLgT;
      computeSize();
      computeNeutral();
    }

    LiegroupSpacePtr_t LiegroupSpace::operator*= (const LiegroupSpaceConstPtr_t& o)
    {
      liegroupTypes_.insert (liegroupTypes_.end (),
          o->liegroupTypes_.begin (),
          o->liegroupTypes_.end ());
      nqs_.insert (nqs_.end (), o->nqs_.begin (), o->nqs_.end ());
      nvs_.insert (nvs_.end (), o->nvs_.begin (), o->nvs_.end ());
      nq_ += o->nq_;
      nv_ += o->nv_;
      neutral_.conservativeResize (nq_);
      neutral_.tail (o->nq ()) = o->neutral ().vector ();
      return weak_.lock();
    }
  } // namespace pinocchio
} // namespace hpp

namespace boost {
  using namespace hpp::pinocchio;

  LiegroupSpacePtr_t operator*
  (const LiegroupSpaceConstPtr_t& sp1, const LiegroupSpaceConstPtr_t& sp2)
  {
    LiegroupSpacePtr_t res (LiegroupSpace::createCopy (sp1));
    *res *= sp2;
    return res;
  }
} // namespace boost
