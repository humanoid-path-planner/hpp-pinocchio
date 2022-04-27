// Copyright (c) 2017, CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <boost/serialization/export.hpp>
#include <boost/serialization/split_free.hpp>
#include <hpp/pinocchio/liegroup-element.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/util/serialization.hh>

#include "../src/addition-visitor.hh"
#include "../src/log-visitor.hh"
#include "../src/size-visitor.hh"
#include "../src/substraction-visitor.hh"
#include "../src/is-normalized-visitor.hh"

namespace hpp {
namespace pinocchio {
typedef std::vector<LiegroupType> LiegroupTypes;

template <typename vector_type>
LiegroupElementBase<vector_type>& LiegroupElementBase<vector_type>::operator+=(
    vectorIn_t v) {
  assert(this->space_->nv() == v.size());

  liegroupType::AdditionVisitor<vector_type> av(this->value_, v);
  for (LiegroupTypes::const_iterator it = this->space_->liegroupTypes().begin();
       it != this->space_->liegroupTypes().end(); ++it) {
    boost::apply_visitor(av, *it);
  }
  assert(av.iq_ == this->space_->nq());
  assert(av.iv_ == this->space_->nv());
  return *this;
}

template LiegroupElementBase<vector_t>&
    LiegroupElementBase<vector_t>::operator+=(vectorIn_t);
template LiegroupElementBase<vectorOut_t>&
    LiegroupElementBase<vectorOut_t>::operator+=(vectorIn_t);

template <typename vector_type>
LiegroupElement operator+(const LiegroupElementConstBase<vector_type>& e,
                          vectorIn_t v) {
  LiegroupElement result(e);
  result += v;
  return result;
}

template LiegroupElement operator+(const LiegroupElementConstBase<vector_t>& e,
                                   vectorIn_t v);
template LiegroupElement operator+(
    const LiegroupElementConstBase<vectorIn_t>& e, vectorIn_t v);
template LiegroupElement operator+(
    const LiegroupElementConstBase<vectorOut_t>& e, vectorIn_t v);

template <typename vector_type1, typename vector_type2>
vector_t operator-(const LiegroupElementConstBase<vector_type1>& e1,
                   const LiegroupElementConstBase<vector_type2>& e2) {
  assert(e1.space()->nq() == e2.space()->nq());
  vector_t result(e1.space()->nv());

  assert(*e1.space() == *e2.space());

  liegroupType::SubstractionVisitor<vector_type1, vector_type2> sv(
      e1.vector(), e2.vector(), result);
  for (LiegroupTypes::const_iterator it = e1.space()->liegroupTypes().begin();
       it != e1.space()->liegroupTypes().end(); ++it) {
    boost::apply_visitor(sv, *it);
  }
  return result;
}

template vector_t operator-(const LiegroupElementConstBase<vector_t>& e1,
                            const LiegroupElementConstBase<vector_t>& e2);
template vector_t operator-(const LiegroupElementConstBase<vector_t>& e1,
                            const LiegroupElementConstBase<vectorIn_t>& e2);
template vector_t operator-(const LiegroupElementConstBase<vector_t>& e1,
                            const LiegroupElementConstBase<vectorOut_t>& e2);
template vector_t operator-(const LiegroupElementConstBase<vectorIn_t>& e1,
                            const LiegroupElementConstBase<vector_t>& e2);
template vector_t operator-(const LiegroupElementConstBase<vectorIn_t>& e1,
                            const LiegroupElementConstBase<vectorIn_t>& e2);
template vector_t operator-(const LiegroupElementConstBase<vectorIn_t>& e1,
                            const LiegroupElementConstBase<vectorOut_t>& e2);
template vector_t operator-(const LiegroupElementConstBase<vectorOut_t>& e1,
                            const LiegroupElementConstBase<vector_t>& e2);
template vector_t operator-(const LiegroupElementConstBase<vectorOut_t>& e1,
                            const LiegroupElementConstBase<vectorIn_t>& e2);
template vector_t operator-(const LiegroupElementConstBase<vectorOut_t>& e1,
                            const LiegroupElementConstBase<vectorOut_t>& e2);

template <typename vector_type>
bool checkNormalized(const LiegroupElementConstBase<vector_type>& e1,
                  const value_type& eps) {
  bool result = true;

  liegroupType::IsNormalizedVisitor<vector_type> isNormalizedvisitor(
      e1.vector(), eps, result);
  for (LiegroupTypes::const_iterator it = e1.space()->liegroupTypes().begin();
       it != e1.space()->liegroupTypes().end(); ++it) {
    boost::apply_visitor(isNormalizedvisitor, *it);
  }
  return result;
}

template bool checkNormalized(const LiegroupElementConstBase<vector_t>&e1,
                              const value_type& eps);
template bool checkNormalized(const LiegroupElementConstBase<vectorIn_t>&e1,
                              const value_type& eps);
template bool checkNormalized(const LiegroupElementConstBase<vectorOut_t>&e1,
                              const value_type& eps);

template <typename vector_type>
vector_t log(const LiegroupElementConstBase<vector_type>& lge) {
  using liegroupType::LogVisitor;
  vector_t res(lge.space()->nv());
  size_type iq = 0, iv = 0;
  for (LiegroupTypes::const_iterator it = lge.space()->liegroupTypes().begin();
       it != lge.space()->liegroupTypes().end(); ++it) {
    liegroupType::SizeVisitor sizeVisitor;
    boost::apply_visitor(sizeVisitor, *it);
    LogVisitor logVisitor(lge.vector().segment(iq, sizeVisitor.nq),
                          res.segment(iv, sizeVisitor.nv));
    boost::apply_visitor(logVisitor, *it);
    iq += sizeVisitor.nq;
    iv += sizeVisitor.nv;
  }
  return res;
}

template vector_t log(const LiegroupElementConstBase<vector_t>& lge);
template vector_t log(const LiegroupElementConstBase<vectorIn_t>& lge);
template vector_t log(const LiegroupElementConstBase<vectorOut_t>& lge);
}  // namespace pinocchio
}  // namespace hpp

namespace boost {
namespace serialization {
template <class Archive>
void load(Archive& ar, hpp::pinocchio::LiegroupElement& c,
          const unsigned int version) {
  (void)version;
  hpp::pinocchio::LiegroupSpacePtr_t space;
  hpp::pinocchio::vector_t vector;
  ar& make_nvp("space", space);
  hpp::serialization::remove_duplicate::serialize_vector(ar, "vector", vector,
                                                         version);
  c = hpp::pinocchio::LiegroupElement(vector, space);
}
template <class Archive>
void save(Archive& ar, const hpp::pinocchio::LiegroupElement& c,
          const unsigned int version) {
  (void)version;
  ar& make_nvp("space", c.space());
  hpp::serialization::remove_duplicate::save_vector(ar, "vector", c.vector(),
                                                    version);
}
}  // namespace serialization
}  // namespace boost

HPP_SERIALIZATION_SPLIT_FREE_IMPLEMENT(hpp::pinocchio::LiegroupElement)
