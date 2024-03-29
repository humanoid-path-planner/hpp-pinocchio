// Copyright (c) 2017-2018, CNRS
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

#define BOOST_TEST_MODULE tframe

// Specific to CLANG to remove bad C99-extensions warning
#if __clang__
#include <boost/variant.hpp>
#endif

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/test/unit_test.hpp>
#include <hpp/pinocchio/liegroup-element.hh>
#include <hpp/pinocchio/serialization.hh>
#include <sstream>

using boost::assign::list_of;
using hpp::pinocchio::LiegroupElement;
using hpp::pinocchio::LiegroupElementConstRef;
using hpp::pinocchio::LiegroupElementRef;
using hpp::pinocchio::LiegroupSpace;
using hpp::pinocchio::LiegroupSpaceConstPtr_t;
using hpp::pinocchio::LiegroupSpacePtr_t;
using hpp::pinocchio::LiegroupType;
using hpp::pinocchio::size_type;
using hpp::pinocchio::value_type;
using hpp::pinocchio::vector_t;

static bool sameR3xSO3(const LiegroupElement& e1, const LiegroupElement& e2,
                       const value_type& eps) {
  vector_t u1(e1.vector()), u2(e2.vector());
  if ((u2 - u1).norm() < eps) return true;
  u2.tail<4>() *= -1;
  if ((u2 - u1).norm() < eps) return true;
  return false;
}

void cast_into_LiegroupElementRef(LiegroupElementRef ref,
                                  const LiegroupElement& u) {
  BOOST_CHECK_EQUAL(ref.vector(), u.vector());
  BOOST_CHECK_EQUAL(ref.vector().data(), u.vector().data());
}

void cast_into_LiegroupElementConstRef(LiegroupElementConstRef ref,
                                       const LiegroupElement& u) {
  BOOST_CHECK_EQUAL(ref.vector(), u.vector());
  BOOST_CHECK_EQUAL(ref.vector().data(), u.vector().data());
}

BOOST_AUTO_TEST_CASE(casting) {
  size_type n(10);
  vector_t u(vector_t::Ones(n));
  LiegroupElement e(u);

  cast_into_LiegroupElementRef(e, e);
  cast_into_LiegroupElementConstRef(e, e);

  // Test assignment operator
  e = vector_t::Zero(n);
  e = LiegroupSpace::R2()->element(vector_t::Ones(2));
}

// Test that operator+ and operator- behave as expected for vector spaces
BOOST_AUTO_TEST_CASE(testVectorSpace) {
  // Test vectors
  size_type n(10);
  vector_t u1(n), u2(n);

  for (std::size_t i = 0; i < 100; ++i) {
    u1.setRandom();
    u2.setRandom();

    vector_t sum(u1 + u2);
    vector_t diff(u1 - u2);

    LiegroupElement e1(u1), e2(u2);
    BOOST_CHECK(e1.space()->nq() == n);
    BOOST_CHECK(e1.space()->nv() == n);
    BOOST_CHECK(e1.space()->nq(0) == n);
    BOOST_CHECK(e1.space()->nv(0) == n);
    BOOST_CHECK(e1.vector().rows() == n);
    BOOST_CHECK(e1.vector().rows() == n);
    BOOST_CHECK((e1 + u2).vector().size() == n);
    BOOST_CHECK((e1 - e2).size() == n);
    BOOST_CHECK(((e1 + u2).vector() - sum).norm() < 1e-10);
    BOOST_CHECK(((e1 - e2) - diff).norm() < 1e-10);
  }
}

// Test a combination of R^3 and SO3
BOOST_AUTO_TEST_CASE(testR3SO3) {
  vector_t u1(7), u3(7);
  vector_t velocity(6);
  LiegroupSpacePtr_t R3xSO3(LiegroupSpace::R3xSO3());

  // Test sizes
  BOOST_CHECK(R3xSO3->nq() == 7);
  BOOST_CHECK(R3xSO3->nv() == 6);
  BOOST_CHECK(R3xSO3->nq(0) == 7);
  BOOST_CHECK(R3xSO3->nv(0) == 6);
  // Test operator==
  BOOST_CHECK(*(R3xSO3) == *(R3xSO3));
  vector_t neutral(7);
  neutral << 0, 0, 0, 0, 0, 0, 1;
  BOOST_CHECK(R3xSO3->neutral().vector() == neutral);
  LiegroupElement e(R3xSO3);
  e.setNeutral();
  BOOST_CHECK(e.vector() == neutral);

  for (std::size_t i = 0; i < 100; ++i) {
    u1.setRandom();
    u3.setRandom();

    u1.tail<4>().normalize();
    u3.tail<4>().normalize();

    LiegroupElement e1(u1, R3xSO3);
    velocity.setRandom();

    LiegroupElement e2(e1 + velocity);

    BOOST_CHECK(((e2 - e1) - velocity).norm() < 1e-10);

    LiegroupElement e3(u3, R3xSO3);
    velocity = e3 - e1;

    BOOST_CHECK(sameR3xSO3(e1 + velocity, e3, 1e-10));

    e2 = e1 + velocity;
    e3 = e2 + (-velocity);

    BOOST_CHECK(sameR3xSO3(e1, e3, 1e-10));
  }

  // Test rotation around frame axes from unit quaternion.
  vector_t u5(7);
  vector_t res(7);

  u5 << 0, 0, 0, 0, 0, 0, 1;
  res << 0, 0, 0, sqrt(2) / 2, 0, 0, sqrt(2) / 2;
  velocity.setZero();
  velocity[3] = M_PI / 2;

  LiegroupElement e5(u5, R3xSO3);

  LiegroupElement e6(e5 + velocity);
  BOOST_CHECK((e6.vector() - res).norm() < 1e-10);

  res << 0, 0, 0, 0, sqrt(2) / 2, 0, sqrt(2) / 2;
  velocity.setZero();
  velocity[4] = M_PI / 2;

  e5 = LiegroupElement(u5, R3xSO3);

  e6 = e5 + velocity;
  BOOST_CHECK((e6.vector() - res).norm() < 1e-10);
}

BOOST_AUTO_TEST_CASE(comparison) {
  BOOST_CHECK(*(LiegroupSpace::Rn(3)) == *(LiegroupSpace::R3()));
  BOOST_CHECK(*(LiegroupSpace::Rn(2)) == *(LiegroupSpace::R2()));
  BOOST_CHECK(*(LiegroupSpace::Rn(1)) == *(LiegroupSpace::R1()));
  BOOST_CHECK(*(LiegroupSpace::R2xSO2()) == *(LiegroupSpace::R2xSO2()));
  BOOST_CHECK(*(LiegroupSpace::R3xSO3()) == *(LiegroupSpace::R3xSO3()));

  BOOST_CHECK(*(LiegroupSpace::Rn(3)) != *(LiegroupSpace::R2()));
  BOOST_CHECK(*(LiegroupSpace::R2()) != *(LiegroupSpace::Rn(3)));
  BOOST_CHECK(*(LiegroupSpace::Rn(3)) != *(LiegroupSpace::Rn(2)));
  BOOST_CHECK(*(LiegroupSpace::Rn(2)) != *(LiegroupSpace::Rn(3)));
  BOOST_CHECK(*(LiegroupSpace::R2xSO2()) != *(LiegroupSpace::R3xSO3()));
  BOOST_CHECK(*(LiegroupSpace::R3xSO3()) != *(LiegroupSpace::R2xSO2()));
  BOOST_CHECK(*(LiegroupSpace::R3xSO3()) != *(LiegroupSpace::R3()));
  BOOST_CHECK(*(LiegroupSpace::R3()) != *(LiegroupSpace::R3xSO3()));
  BOOST_CHECK(*(LiegroupSpace::R3xSO3()) != *(LiegroupSpace::Rn(3)));
  BOOST_CHECK(*(LiegroupSpace::Rn(3)) != *(LiegroupSpace::R3xSO3()));
  BOOST_CHECK(*(LiegroupSpace::R2xSO2()) != *(LiegroupSpace::Rn(3)));
  BOOST_CHECK(*(LiegroupSpace::Rn(3)) != *(LiegroupSpace::R2xSO2()));
  BOOST_CHECK(*(LiegroupSpace::R2xSO2()) != *(LiegroupSpace::Rn(2)));
  BOOST_CHECK(*(LiegroupSpace::Rn(2)) != *(LiegroupSpace::R2xSO2()));

  BOOST_CHECK(LiegroupSpace::R1()->isVectorSpace());
  BOOST_CHECK(LiegroupSpace::R2()->isVectorSpace());
  BOOST_CHECK(LiegroupSpace::R3()->isVectorSpace());
  BOOST_CHECK(LiegroupSpace::Rn(4)->isVectorSpace());
  BOOST_CHECK(LiegroupSpace::empty()->isVectorSpace());
  BOOST_CHECK(!LiegroupSpace::SE2()->isVectorSpace());
  BOOST_CHECK(!LiegroupSpace::SE3()->isVectorSpace());
  BOOST_CHECK(!LiegroupSpace::R2xSO2()->isVectorSpace());
  BOOST_CHECK(!LiegroupSpace::R3xSO3()->isVectorSpace());
}

BOOST_AUTO_TEST_CASE(multiplication) {
  LiegroupSpacePtr_t sp(LiegroupSpace::Rn(0) * LiegroupSpace::Rn(10) *
                        LiegroupSpace::R3() * LiegroupSpace::R3xSO3());
  vector_t n;
  n.resize(20);
  n.setZero();
  n[19] = 1;
  BOOST_CHECK_EQUAL(sp->nq(), 20);
  BOOST_CHECK_EQUAL(sp->nv(), 19);
  BOOST_CHECK_EQUAL(sp->liegroupTypes().size(), 2);
  BOOST_CHECK_EQUAL(sp->nq(0), 13);
  BOOST_CHECK_EQUAL(sp->nv(0), 13);
  BOOST_CHECK_EQUAL(sp->nq(1), 7);
  BOOST_CHECK_EQUAL(sp->nv(1), 6);
  BOOST_CHECK_EQUAL(sp->name(), "R^13*R^3*SO(3)");
  BOOST_CHECK_EQUAL(sp->neutral().vector(), n);
  BOOST_CHECK_EQUAL(sp->neutral().space(), sp);
  BOOST_CHECK_EQUAL(sp, sp->neutral().space());
}

BOOST_AUTO_TEST_CASE(Cartesian_power_1) {
  LiegroupSpaceConstPtr_t sp1(LiegroupSpace::Rn(10));
  LiegroupSpacePtr_t sp2(sp1 ^ 4);

  BOOST_CHECK_EQUAL(sp2->nq(), 40);
  BOOST_CHECK_EQUAL(sp2->nv(), 40);
  BOOST_CHECK_EQUAL(sp2->liegroupTypes().size(), 1);
  BOOST_CHECK_EQUAL(sp2->nq(0), 40);
  BOOST_CHECK_EQUAL(sp2->nv(0), 40);
  BOOST_CHECK_EQUAL(sp2->name(), "R^40");
}

BOOST_AUTO_TEST_CASE(Cartesian_power_2) {
  LiegroupSpaceConstPtr_t sp1(LiegroupSpace::SE3() * LiegroupSpace::Rn(10));
  LiegroupSpacePtr_t sp2(sp1 ^ 6);

  BOOST_CHECK_EQUAL(sp2->nq(), 17 * 6);
  BOOST_CHECK_EQUAL(sp2->nv(), 16 * 6);
  BOOST_CHECK_EQUAL(sp2->liegroupTypes().size(), 2 * 6);
  for (std::size_t i = 0; i < 6; ++i) {
    BOOST_CHECK_EQUAL(sp2->nq(2 * i), 7);
    BOOST_CHECK_EQUAL(sp2->nv(2 * i), 6);
    BOOST_CHECK_EQUAL(sp2->nq(2 * i + 1), 10);
    BOOST_CHECK_EQUAL(sp2->nv(2 * i + 1), 10);
  }
  BOOST_CHECK_EQUAL(sp2->name(),
                    "SE(3)*R^10*SE(3)*R^10*SE(3)*R^10*"
                    "SE(3)*R^10*SE(3)*R^10*SE(3)*R^10");
}

BOOST_AUTO_TEST_CASE(Cartesian_power_3) {
  LiegroupSpaceConstPtr_t sp1(LiegroupSpace::SE3() * LiegroupSpace::Rn(10));
  LiegroupSpacePtr_t sp2(sp1 ^ 0);
  LiegroupSpacePtr_t sp3(sp1 ^ 1);

  BOOST_CHECK_EQUAL(*sp1, *sp3);
  BOOST_CHECK_EQUAL(*sp2, *(LiegroupSpace::empty()));
}

BOOST_AUTO_TEST_CASE(log_) {
  size_type n(10);
  vector_t u;
  u.resize(n);

  for (std::size_t i = 0; i < 100; ++i) {
    u.setRandom();
    LiegroupElement e(u, LiegroupSpace::Rn(n));
    BOOST_CHECK(hpp::pinocchio::log(e) == u);
  }

  LiegroupSpacePtr_t R6xSO3(LiegroupSpace::R3() * LiegroupSpace::R3xSO3());
  LiegroupElement e(R6xSO3);
  e.setNeutral();

  BOOST_CHECK(R6xSO3->nq() == 10);
  BOOST_CHECK(R6xSO3->nv() == 9);
  BOOST_CHECK(R6xSO3->nq(0) == 3);
  BOOST_CHECK(R6xSO3->nv(0) == 3);
  BOOST_CHECK(R6xSO3->nq(1) == 7);
  BOOST_CHECK(R6xSO3->nv(1) == 6);

  BOOST_CHECK((hpp::pinocchio::log(e).isZero(1e-10)));
}

template <typename base_archive = boost::archive::xml_oarchive>
struct oarchive : base_archive,
                  hpp::serialization::remove_duplicate::vector_archive {
  oarchive(std::ostream& is) : base_archive(is) {}
};

void test_serialization(LiegroupSpacePtr_t space) {
  LiegroupElement e(space);
  e.setNeutral();

  std::stringstream ss;
  {
    oarchive<boost::archive::xml_oarchive> oa(ss);
    oa << boost::serialization::make_nvp("element", e);
  }

  BOOST_TEST_MESSAGE(ss.str());

  LiegroupElement e2;
  {
    boost::archive::xml_iarchive ia(ss);
    ia >> boost::serialization::make_nvp("element", e2);
  }

  BOOST_CHECK_EQUAL(*e.space(), *e2.space());
  BOOST_CHECK(e.vector() == e2.vector());
}

BOOST_AUTO_TEST_CASE(serialization) {
  test_serialization(LiegroupSpace::R1());
  test_serialization(LiegroupSpace::R2());
  test_serialization(LiegroupSpace::R3());
  test_serialization(LiegroupSpace::Rn(4));
  test_serialization(LiegroupSpace::SO2());
  test_serialization(LiegroupSpace::SO3());
  test_serialization(LiegroupSpace::SE2());
  test_serialization(LiegroupSpace::SE3());
  test_serialization(LiegroupSpace::R2xSO2());
  test_serialization(LiegroupSpace::R3xSO3());
  test_serialization(LiegroupSpace::R3() * LiegroupSpace::R3xSO3());
  test_serialization(LiegroupSpace::R3() * LiegroupSpace::R3xSO3());
}
