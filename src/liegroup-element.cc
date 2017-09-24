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

namespace hpp {
  namespace pinocchio {
    namespace liegroupType {
      struct DimensionVisitor : public boost::static_visitor <>
      {
        template <typename LiegroupType> void operator () (LiegroupType& op)
        {
          nq = op.nq ();
          nv = op.nv ();
        }
        size_type nq, nv;
      }; // struct DimensionVisitor

      struct AdditionVisitor : public boost::static_visitor <>
      {
        AdditionVisitor (const vector_t& e, const vector_t& v) :
          e_ (e), v_ (v), result (e.size ())
        {
        }
        template <typename LiegroupType> void operator () (LiegroupType& op)
        {
          op.integrate_impl (e_, v_, result);
        }
        vector_t e_;
        vector_t v_;
        vector_t result;
      }; // struct AdditionVisitor

      struct SubstractionVisitor : public boost::static_visitor <>
      {
        SubstractionVisitor (const vector_t& e1, const vector_t& e2,
                             const size_type& nv) :
          e1_ (e1), e2_ (e2), result (nv)
        {
        }
        template <typename LiegroupType> void operator () (LiegroupType& op)
        {
          op.difference_impl (e2_, e1_, result);
        }
        vector_t e1_, e2_;
        vector_t result;
      }; // struct SubstractionVisitor
    } // namespace liegroupType

    void LiegroupElement::setNeutral ()
    {
      value_ = space_->neutral ().vector ();
    }

    LiegroupElement operator+ (const LiegroupElement& e, const vector_t& v)
    {
      assert (e.space ()->nv () == v.size ());
      typedef std::vector <LiegroupType> LiegroupTypes;
      LiegroupElement result (e);
      size_type iq = 0, iv = 0;
      for (LiegroupTypes::const_iterator it =
             e.space ()->liegroupTypes ().begin ();
           it != e.space ()->liegroupTypes ().end (); ++it) {
        liegroupType::DimensionVisitor dv;
        boost::apply_visitor (dv, *it);

        liegroupType::AdditionVisitor av (e.value_.segment (iq, dv.nq),
                                          v.segment (iv, dv.nv));
        boost::apply_visitor (av, *it);
        result.value_.segment (iq, dv.nq) = av.result;
        iq += dv.nq;
        iv += dv.nv;
      }
      return result;
    }

    vector_t operator- (const LiegroupElement& e1, const LiegroupElement& e2)
    {
      assert (e1.space ()->nq () == e2.space ()->nq ());
      typedef std::vector <LiegroupType> LiegroupTypes;
      vector_t result (e1.space ()->nv ());
      size_type iq = 0, iv = 0;

      LiegroupTypes::const_iterator it1 =
        e1.space ()->liegroupTypes ().begin ();
      LiegroupTypes::const_iterator it2 =
        e2.space ()->liegroupTypes ().begin ();

      while ((it1 != e1.space ()->liegroupTypes ().end ()) &&
             (it2 != e2.space ()->liegroupTypes ().end ())) {
        liegroupType::DimensionVisitor dv;
        boost::apply_visitor (dv, *it1);

        liegroupType::SubstractionVisitor sv (e1.value_.segment (iq, dv.nq),
                                              e2.value_.segment (iq, dv.nq),
                                              e1.space ()->nv ());
        boost::apply_visitor (sv, *it1);
        result.segment (iv, dv.nv) = sv.result;
        iq += dv.nq;
        iv += dv.nv;
        ++it1; ++it2;
      }
      return result;
    }

  } // namespace pinocchio
} // namespace hpp
