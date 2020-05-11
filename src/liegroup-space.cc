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

#include <hpp/pinocchio/liegroup-space.hh>

#include <boost/serialization/variant.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <pinocchio/serialization/eigen.hpp>

#include <hpp/util/serialization.hh>

#include <hpp/pinocchio/liegroup-element.hh>
#include <hpp/pinocchio/liegroup/serialization.hh>

#include "../src/comparison.hh"
#include "../src/size-visitor.hh"
#include "../src/dintegrate-visitor.hh"
#include "../src/jdifference-visitor.hh"
#include "../src/visitor/interpolate.hh"

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
    LiegroupSpacePtr_t LiegroupSpace::R1 (bool rotation)
    {
      LiegroupSpace* ptr;
      if (rotation)
      {
        ptr = new LiegroupSpace(liegroup::VectorSpaceOperation <1, true> ());
      }
      else
      {
        ptr = new LiegroupSpace(liegroup::VectorSpaceOperation <1, false> ());
      }
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

    /// Return \f$SO(2)\f$
    LiegroupSpacePtr_t LiegroupSpace::SO2 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace
                          (liegroup::SpecialOrthogonalOperation<2> ()));
      LiegroupSpacePtr_t  shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    /// Return \f$SO(3)\f$
    LiegroupSpacePtr_t LiegroupSpace::SO3 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace
                          (liegroup::SpecialOrthogonalOperation<3> ()));
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
                          (liegroup::SpecialEuclideanOperation <2>()));
      LiegroupSpacePtr_t  shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    /// Return \f$SE(3)\f$
    LiegroupSpacePtr_t LiegroupSpace::SE3 ()
    {
      LiegroupSpace* ptr (new LiegroupSpace
                          (liegroup::SpecialEuclideanOperation <3>()));
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

    size_type LiegroupSpace::nq (const std::size_t& rank) const
    {
      assert (rank < liegroupTypes_.size());
      liegroupType::SizeVisitor v;
      boost::apply_visitor (v, liegroupTypes_[rank]);
      return v.nq;
    }

    size_type LiegroupSpace::nv (const std::size_t& rank) const
    {
      assert (rank < liegroupTypes_.size());
      liegroupType::SizeVisitor v;
      boost::apply_visitor (v, liegroupTypes_[rank]);
      return v.nv;
    }

    LiegroupElement LiegroupSpace::neutral () const
    {
      return LiegroupElement (neutral_, weak_.lock ());
    }

    LiegroupElement LiegroupSpace::element (vectorIn_t q) const
    {
      return LiegroupElement (q, weak_.lock ());
    }

    LiegroupElementRef LiegroupSpace::elementRef (vectorOut_t q) const
    {
      return LiegroupElementRef (q, weak_.lock ());
    }

    LiegroupElementConstRef LiegroupSpace::elementConstRef (vectorIn_t q) const
    {
      return LiegroupElementConstRef (q, weak_.lock ());
    }

    LiegroupElement LiegroupSpace::exp (vectorIn_t v) const
    {
      return neutral () + v;
    }

    template <DerivativeProduct side>
    void LiegroupSpace::dIntegrate_dq (LiegroupElementConstRef q, vectorIn_t v, matrixOut_t Jq) const
    {
      assert (q.size() == nq());
      assert (v.size() == nv());
      assert (Jq.rows() == nv());
      size_type row = 0;
      size_type configRow = 0;
      liegroupType::dIntegrateVisitor<0, side> jiv (q.vector(), v, Jq, row, configRow);
      for (std::size_t i = 0; i < liegroupTypes_.size (); ++i) {
        boost::apply_visitor (jiv, liegroupTypes_ [i]);
      }
      assert (row == nv());
      assert (configRow == nq());
    }

    template <DerivativeProduct side>
    void LiegroupSpace::dIntegrate_dv (LiegroupElementConstRef q, vectorIn_t v, matrixOut_t Jv) const
    {
      assert (q.size() == nq());
      assert (v.size() == nv());
      assert (Jv.rows() == nv());
      size_type row = 0;
      size_type configRow = 0;
      liegroupType::dIntegrateVisitor<1, side> jiv (q.vector(), v, Jv, row, configRow);
      for (std::size_t i = 0; i < liegroupTypes_.size (); ++i) {
        boost::apply_visitor (jiv, liegroupTypes_ [i]);
      }
      assert (row == nv());
      assert (configRow == nq());
    }

    template void LiegroupSpace::dIntegrate_dq<DerivativeTimesInput> (LiegroupElementConstRef, vectorIn_t, matrixOut_t) const;
    template void LiegroupSpace::dIntegrate_dq<InputTimesDerivative> (LiegroupElementConstRef, vectorIn_t, matrixOut_t) const;
    template void LiegroupSpace::dIntegrate_dv<DerivativeTimesInput> (LiegroupElementConstRef, vectorIn_t, matrixOut_t) const;
    template void LiegroupSpace::dIntegrate_dv<InputTimesDerivative> (LiegroupElementConstRef, vectorIn_t, matrixOut_t) const;

    template <bool ApplyOnTheLeft>
    void LiegroupSpace::Jdifference (vectorIn_t q0, vectorIn_t q1, matrixOut_t J0, matrixOut_t J1) const
    {
      dDifference_dq0<ApplyOnTheLeft?DerivativeTimesInput:InputTimesDerivative> (q0, q1, J0);
      dDifference_dq1<ApplyOnTheLeft?DerivativeTimesInput:InputTimesDerivative> (q0, q1, J1);
    }

    template <DerivativeProduct side>
    void LiegroupSpace::dDifference_dq0 (vectorIn_t q0, vectorIn_t q1, matrixOut_t J0) const
    {
      assert (q0.size() == nq() && q1.size() == nq());

      liegroupType::dDifferenceVisitor<ARG0, side> jdv (q0,q1,J0);
      for (std::size_t i = 0; i < liegroupTypes_.size (); ++i) {
        boost::apply_visitor (jdv, liegroupTypes_ [i]);
      }
      assert (jdv.iq_ == nq());
      assert (jdv.iv_ == nv());
    }

    template <DerivativeProduct side>
    void LiegroupSpace::dDifference_dq1 (vectorIn_t q0, vectorIn_t q1, matrixOut_t J1) const
    {
      assert (q0.size() == nq() && q1.size() == nq());

      liegroupType::dDifferenceVisitor<ARG1, side> jdv (q0,q1,J1);
      for (std::size_t i = 0; i < liegroupTypes_.size (); ++i) {
        boost::apply_visitor (jdv, liegroupTypes_ [i]);
      }
      assert (jdv.iq_ == nq());
      assert (jdv.iv_ == nv());
    }

    template void LiegroupSpace::Jdifference<true > (vectorIn_t q0, vectorIn_t q1, matrixOut_t J0, matrixOut_t J1) const;
    template void LiegroupSpace::Jdifference<false> (vectorIn_t q0, vectorIn_t q1, matrixOut_t J0, matrixOut_t J1) const;
    template void LiegroupSpace::dDifference_dq0<DerivativeTimesInput> (vectorIn_t, vectorIn_t, matrixOut_t) const;
    template void LiegroupSpace::dDifference_dq0<InputTimesDerivative> (vectorIn_t, vectorIn_t, matrixOut_t) const;
    template void LiegroupSpace::dDifference_dq1<DerivativeTimesInput> (vectorIn_t, vectorIn_t, matrixOut_t) const;
    template void LiegroupSpace::dDifference_dq1<InputTimesDerivative> (vectorIn_t, vectorIn_t, matrixOut_t) const;

    void LiegroupSpace::interpolate (vectorIn_t q0, vectorIn_t q1, value_type u,
        vectorOut_t r) const
    {
      assert (q0.size() == nq());
      assert (q1.size() == nq());
      assert (r .size() == nq());

      liegroupType::visitor::Interpolate iv (q0, q1, u, r);
      for (LiegroupTypes::const_iterator it = liegroupTypes ().begin ();
           it != liegroupTypes ().end (); ++it)
        boost::apply_visitor (iv, *it);
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
        bool res = boost::apply_visitor (v, *it1);
        if (!res) return false;
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
      nq_ (0), nv_ (0), neutral_ (), weak_ ()
    {}

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

    void LiegroupSpace::mergeVectorSpaces ()
    {
      if (liegroupTypes_.empty()) return;

      LiegroupTypes newLgT;

      liegroupType::IsVectorSpace ivs;
      liegroupType::SizeVisitor curSizes;
      size_type vsSize = 0;
      bool prevIsVectorSpace = false;

      for (LiegroupTypes::const_iterator _cur = liegroupTypes_.begin();
          _cur != liegroupTypes_.end (); ++_cur) {
        boost::apply_visitor (ivs, *_cur);
        bool curIsVectorSpace = ivs.isVectorSpace;
        if (curIsVectorSpace) {
          boost::apply_visitor (curSizes, *_cur);
        }
        if (prevIsVectorSpace && curIsVectorSpace) {
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
          if (curIsVectorSpace) {
            vsSize = curSizes.nq;
            prevIsVectorSpace = true;
          } else {
            vsSize = 0;
            prevIsVectorSpace = false;
          }
          newLgT.push_back (*_cur);
        }
      }
      liegroupTypes_ = newLgT;
      computeSize();
      computeNeutral();
    }

    LiegroupSpacePtr_t LiegroupSpace::vectorSpacesMerged () const
    {
      LiegroupSpacePtr_t other (createCopy(weak_.lock()));
      other->mergeVectorSpaces();
      return other;
    }

    bool LiegroupSpace::isVectorSpace () const
    {
      liegroupType::IsVectorSpace ivs;
      for (LiegroupTypes::const_iterator _cur = liegroupTypes_.begin();
          _cur != liegroupTypes_.end (); ++_cur) {
        boost::apply_visitor (ivs, *_cur);
        if (!ivs.isVectorSpace) return false;
      }
      return true;
    }

    LiegroupSpacePtr_t LiegroupSpace::operator*= (const LiegroupSpaceConstPtr_t& o)
    {
      liegroupTypes_.insert (liegroupTypes_.end (),
          o->liegroupTypes_.begin (),
          o->liegroupTypes_.end ());
      nq_ += o->nq_;
      nv_ += o->nv_;
      neutral_.conservativeResize (nq_);
      neutral_.tail (o->nq ()) = o->neutral ().vector ();
      mergeVectorSpaces ();
      return weak_.lock();
    }

    template<class Archive>
    void LiegroupSpace::serialize(Archive & ar, const unsigned int version)
    {
      (void) version;
      ar & BOOST_SERIALIZATION_NVP(liegroupTypes_);
      ar & BOOST_SERIALIZATION_NVP(nq_);
      ar & BOOST_SERIALIZATION_NVP(nv_);
      ar & BOOST_SERIALIZATION_NVP(neutral_);
      ar & BOOST_SERIALIZATION_NVP(weak_);
    }

    HPP_SERIALIZATION_IMPLEMENT(LiegroupSpace);
  } // namespace pinocchio
} // namespace hpp

BOOST_CLASS_EXPORT(hpp::pinocchio::LiegroupSpace)

namespace boost {
  using namespace hpp::pinocchio;

  LiegroupSpacePtr_t operator*
  (const LiegroupSpaceConstPtr_t& sp1, const LiegroupSpaceConstPtr_t& sp2)
  {
    LiegroupSpacePtr_t res (LiegroupSpace::createCopy (sp1));
    *res *= sp2;
    res->mergeVectorSpaces ();
    return res;
  }
  /// Cartesian power by an integer
  LiegroupSpacePtr_t operator^ (const LiegroupSpaceConstPtr_t& sp, size_type n)
  {
    assert (n >= 0);
    if (n==0) return LiegroupSpace::empty ();
    LiegroupSpacePtr_t result (LiegroupSpace::createCopy (sp)); --n;
    while (n > 0) {
      result = result * sp; --n;
    }
    result->mergeVectorSpaces ();
    return result;
  }
} // namespace boost
