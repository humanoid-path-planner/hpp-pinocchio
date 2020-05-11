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

#include <boost/serialization/export.hpp>
#include <boost/serialization/split_free.hpp>

#include <hpp/util/serialization.hh>

#include <pinocchio/serialization/eigen.hpp>

#include "../src/size-visitor.hh"
#include "../src/addition-visitor.hh"
#include "../src/substraction-visitor.hh"
#include "../src/log-visitor.hh"

namespace hpp {
  namespace pinocchio {
    typedef std::vector <LiegroupType> LiegroupTypes;

    template <typename vector_type>
    LiegroupElementBase<vector_type>& LiegroupElementBase<vector_type>::operator+= (vectorIn_t v)
    {
      assert (this->space_->nv () == v.size ());

      liegroupType::AdditionVisitor<vector_type> av (this->value_, v);
      for (LiegroupTypes::const_iterator it = this->space_->liegroupTypes ().begin ();
           it != this->space_->liegroupTypes ().end (); ++it) {
        boost::apply_visitor (av, *it);
      }
      assert (av.iq_ == this->space_->nq());
      assert (av.iv_ == this->space_->nv());
      return *this;
    }

    template LiegroupElementBase<vector_t   >& LiegroupElementBase<vector_t   >::operator+= (vectorIn_t);
    template LiegroupElementBase<vectorOut_t>& LiegroupElementBase<vectorOut_t>::operator+= (vectorIn_t);

    template <typename vector_type>
    LiegroupElement operator+ (const LiegroupElementConstBase<vector_type>& e, vectorIn_t v)
    {
      LiegroupElement result (e);
      result += v;
      return result;
    }

    template LiegroupElement operator+ (const LiegroupElementConstBase<vector_t   >& e, vectorIn_t v);
    template LiegroupElement operator+ (const LiegroupElementConstBase<vectorIn_t >& e, vectorIn_t v);
    template LiegroupElement operator+ (const LiegroupElementConstBase<vectorOut_t>& e, vectorIn_t v);

    template <typename vector_type1, typename vector_type2>
    vector_t operator- (const LiegroupElementConstBase<vector_type1>& e1, const LiegroupElementConstBase<vector_type2>& e2)
    {
      assert (e1.space ()->nq () == e2.space ()->nq ());
      vector_t result (e1.space ()->nv ());

      assert (*e1.space() == *e2.space());

      liegroupType::SubstractionVisitor<vector_type1, vector_type2>
        sv (e1.vector(), e2.vector(), result);
      for (LiegroupTypes::const_iterator it = e1.space()->liegroupTypes ().begin ();
           it != e1.space()->liegroupTypes ().end (); ++it) {
        boost::apply_visitor (sv, *it);
      }
      return result;
    }

    template vector_t operator- (const LiegroupElementConstBase<vector_t   >& e1, const LiegroupElementConstBase<vector_t   >& e2);
    template vector_t operator- (const LiegroupElementConstBase<vector_t   >& e1, const LiegroupElementConstBase<vectorIn_t >& e2);
    template vector_t operator- (const LiegroupElementConstBase<vector_t   >& e1, const LiegroupElementConstBase<vectorOut_t>& e2);
    template vector_t operator- (const LiegroupElementConstBase<vectorIn_t >& e1, const LiegroupElementConstBase<vector_t   >& e2);
    template vector_t operator- (const LiegroupElementConstBase<vectorIn_t >& e1, const LiegroupElementConstBase<vectorIn_t >& e2);
    template vector_t operator- (const LiegroupElementConstBase<vectorIn_t >& e1, const LiegroupElementConstBase<vectorOut_t>& e2);
    template vector_t operator- (const LiegroupElementConstBase<vectorOut_t>& e1, const LiegroupElementConstBase<vector_t   >& e2);
    template vector_t operator- (const LiegroupElementConstBase<vectorOut_t>& e1, const LiegroupElementConstBase<vectorIn_t >& e2);
    template vector_t operator- (const LiegroupElementConstBase<vectorOut_t>& e1, const LiegroupElementConstBase<vectorOut_t>& e2);

    template <typename vector_type>
    vector_t log (const LiegroupElementConstBase<vector_type>& lge)
    {
      using liegroupType::LogVisitor;
      vector_t res (lge.space ()->nv ());
      size_type iq = 0, iv = 0;
      for (LiegroupTypes::const_iterator it =
           lge.space ()->liegroupTypes ().begin ();
           it != lge.space ()->liegroupTypes ().end (); ++it) {
        liegroupType::SizeVisitor sizeVisitor;
        boost::apply_visitor (sizeVisitor, *it);
        LogVisitor logVisitor (lge.vector ().segment (iq, sizeVisitor.nq),
                               res.segment (iv, sizeVisitor.nv));
        boost::apply_visitor (logVisitor, *it);
        iq += sizeVisitor.nq; iv += sizeVisitor.nv;
      }
      return res;
    }

    template vector_t log (const LiegroupElementConstBase<vector_t   >& lge);
    template vector_t log (const LiegroupElementConstBase<vectorIn_t >& lge);
    template vector_t log (const LiegroupElementConstBase<vectorOut_t>& lge);
  } // namespace pinocchio
} // namespace hpp

namespace boost {
namespace serialization {
template<class Archive>
void load (Archive & ar, hpp::pinocchio::LiegroupElement& c, const unsigned int version)
{
  (void) version;
  hpp::pinocchio::LiegroupSpacePtr_t space;
  hpp::pinocchio::vector_t vector;
  ar & make_nvp("space", space);
  ar & make_nvp("vector", vector);
  c = hpp::pinocchio::LiegroupElement(vector, space);
}
template<class Archive>
void save (Archive & ar, const hpp::pinocchio::LiegroupElement& c, const unsigned int version)
{
  (void) version;
  ar & make_nvp("space", c.space());
  ar & make_nvp("vector", c.vector());
}
template<class Archive>
void serialize(Archive & ar, hpp::pinocchio::LiegroupElement& c, const unsigned int file_version)
{
  split_free(ar, c, file_version);
}

using archive::polymorphic_iarchive;
using archive::polymorphic_oarchive;
template void load <polymorphic_iarchive> (polymorphic_iarchive & ar,
    hpp::pinocchio::LiegroupElement& c, const unsigned int version);
template void save <polymorphic_oarchive> (polymorphic_oarchive & ar,
    const hpp::pinocchio::LiegroupElement& c, const unsigned int version);

template void serialize <polymorphic_iarchive> (polymorphic_iarchive & ar,
    hpp::pinocchio::LiegroupElement& c, const unsigned int version);
template void serialize <polymorphic_oarchive> (polymorphic_oarchive & ar,
    hpp::pinocchio::LiegroupElement& c, const unsigned int version);
} // namespace serialization
} // namespace boost
