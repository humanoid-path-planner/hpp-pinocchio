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

#define BOOST_TEST_MODULE tframe
#include <boost/test/unit_test.hpp>
#include <boost/assign/list_of.hpp>
#include <hpp/pinocchio/liegroup-element.hh>

using boost::assign::list_of;
using hpp::pinocchio::size_type;
using hpp::pinocchio::value_type;
using hpp::pinocchio::vector_t;
using hpp::pinocchio::LiegroupElement;
using hpp::pinocchio::LiegroupType;
using hpp::pinocchio::LiegroupSpace;
using hpp::pinocchio::LiegroupSpacePtr_t;

static bool sameR3xSO3 (const LiegroupElement& e1, const LiegroupElement& e2,
                        const value_type& eps)
{
  vector_t u1 (e1. vector ()), u2 (e2. vector ());
  if ((u2-u1).norm () < eps) return true;
  u2.tail <4> () *= -1;
  if ((u2-u1).norm () < eps) return true;
  return false;
}

// Test that operator+ and operator- behave as expected for vector spaces
BOOST_AUTO_TEST_CASE (testVectorSpace)
{
  // Test vectors
  size_type n (10);
  vector_t u1 (n), u2 (n);

  for (std::size_t i=0; i<100; ++i) {
    u1.setRandom ();
    u2.setRandom ();

    vector_t sum (u1 + u2);
    vector_t diff (u1 - u2);

    LiegroupElement e1 (u1), e2 (u2);
    BOOST_CHECK (e1. vector ().rows () == n);
    BOOST_CHECK (e1. vector ().rows () == n);
    BOOST_CHECK ((e1 + u2). vector ().size () == n);
    BOOST_CHECK ((e1 - e2).size () == n);
    BOOST_CHECK (((e1 + u2). vector () - sum).norm () < 1e-10);
    BOOST_CHECK (((e1 - e2) - diff).norm () < 1e-10);
  }
}

// Test a combination of R^3 and SO3
BOOST_AUTO_TEST_CASE (testR3SO3)
{
  vector_t u1 (7), u3 (7);
  vector_t velocity (6);
  LiegroupSpacePtr_t R3xSO3 (LiegroupSpace::R3xSO3 ());

  // Test operator==
  BOOST_CHECK (*(R3xSO3) == *(R3xSO3));
  vector_t neutral (7); neutral << 0, 0, 0, 0, 0, 0, 1;
  BOOST_CHECK (R3xSO3->neutral ().vector () == neutral);
  LiegroupElement e (R3xSO3);
  e.setNeutral ();
  BOOST_CHECK (e. vector () == neutral);

  for (std::size_t i=0; i<100; ++i) {
    u1.setRandom ();
    u3.setRandom ();

    u1.tail <4> ().normalize ();
    u3.tail <4> ().normalize ();

    LiegroupElement e1 (u1, R3xSO3);
    velocity.setRandom ();

    LiegroupElement e2 (e1 + velocity);

    BOOST_CHECK (((e2 - e1) - velocity).norm () < 1e-10);

    LiegroupElement e3 (u3, R3xSO3);
    velocity = e3 - e1;

    BOOST_CHECK (sameR3xSO3 (e1 + velocity, e3, 1e-10));

    e2 = e1 + velocity;
    e3 = e2 + (- velocity);

    BOOST_CHECK (sameR3xSO3 (e1, e3, 1e-10));
  }

  // Test rotation around frame axes from unit quaternion.
  vector_t u5 (7);
  vector_t res (7);

  u5 << 0, 0, 0, 0, 0, 0, 1;
  res << 0, 0, 0, sqrt (2)/2, 0, 0, sqrt (2)/2;
  velocity.setZero ();
  velocity [3] = M_PI/2;

  LiegroupElement e5 (u5, R3xSO3);

  LiegroupElement e6 (e5 + velocity);
  BOOST_CHECK ((e6. vector () - res).norm () < 1e-10);

  res << 0, 0, 0, 0, sqrt (2)/2, 0, sqrt (2)/2;
  velocity.setZero ();
  velocity [4] = M_PI/2;

  e5 = LiegroupElement (u5, R3xSO3);

  e6 = e5 + velocity;
  BOOST_CHECK ((e6. vector () - res).norm () < 1e-10);
}

BOOST_AUTO_TEST_CASE (comparison)
{
  BOOST_CHECK (*(LiegroupSpace::Rn (3)) == *(LiegroupSpace::R3 ()));
  BOOST_CHECK (*(LiegroupSpace::Rn (2)) == *(LiegroupSpace::R2 ()));
  BOOST_CHECK (*(LiegroupSpace::Rn (1)) == *(LiegroupSpace::R1 ()));
  BOOST_CHECK (*(LiegroupSpace::R2xSO2 ()) == *(LiegroupSpace::R2xSO2 ()));
  BOOST_CHECK (*(LiegroupSpace::R3xSO3 ()) == *(LiegroupSpace::R3xSO3 ()));

  BOOST_CHECK (*(LiegroupSpace::Rn (3)) != *(LiegroupSpace::R2 ()));
  BOOST_CHECK (*(LiegroupSpace::R2 ()) != *(LiegroupSpace::Rn (3)));
  BOOST_CHECK (*(LiegroupSpace::Rn (3)) != *(LiegroupSpace::Rn (2)));
  BOOST_CHECK (*(LiegroupSpace::Rn (2)) != *(LiegroupSpace::Rn (3)));
  BOOST_CHECK (*(LiegroupSpace::R2xSO2 ()) != *(LiegroupSpace::R3xSO3 ()));
  BOOST_CHECK (*(LiegroupSpace::R3xSO3 ()) != *(LiegroupSpace::R2xSO2 ()));
  BOOST_CHECK (*(LiegroupSpace::R3xSO3 ()) != *(LiegroupSpace::R3 ()));
  BOOST_CHECK (*(LiegroupSpace::R3 ()) != *(LiegroupSpace::R3xSO3 ()));
  BOOST_CHECK (*(LiegroupSpace::R3xSO3 ()) != *(LiegroupSpace::Rn (3)));
  BOOST_CHECK (*(LiegroupSpace::Rn (3)) != *(LiegroupSpace::R3xSO3 ()));
  BOOST_CHECK (*(LiegroupSpace::R2xSO2 ()) != *(LiegroupSpace::Rn (3)));
  BOOST_CHECK (*(LiegroupSpace::Rn (3)) != *(LiegroupSpace::R2xSO2 ()));
  BOOST_CHECK (*(LiegroupSpace::R2xSO2 ()) != *(LiegroupSpace::Rn (2)));
  BOOST_CHECK (*(LiegroupSpace::Rn (2)) != *(LiegroupSpace::R2xSO2 ()));
}

BOOST_AUTO_TEST_CASE (multiplication)
{
  LiegroupSpacePtr_t sp (LiegroupSpace::Rn (10) * LiegroupSpace::R3 () *
                         LiegroupSpace::R3xSO3 ());
  vector_t n; n.resize (20); n.setZero (); n [19] = 1;
  BOOST_CHECK (sp->nq () == 20);
  BOOST_CHECK (sp->nv () == 19);
  BOOST_CHECK (sp->name () == "R^10*R^3*R^3*SO(3)");
  BOOST_CHECK (sp->neutral ().vector () == n);
  BOOST_CHECK (sp->neutral ().space () == sp);
  BOOST_CHECK (sp == sp->neutral ().space ());
}

BOOST_AUTO_TEST_CASE (log_)
{
  size_type n (10);
  vector_t u; u.resize (n);

  for (std::size_t i=0; i<100; ++i) {
    u.setRandom ();
    LiegroupElement e (u, LiegroupSpace::Rn (n));
    BOOST_CHECK (hpp::pinocchio::log (e) == u);
  }

  LiegroupSpacePtr_t R6xSO3 (LiegroupSpace::R3 () * LiegroupSpace::R3xSO3 ());
  LiegroupElement e (R6xSO3); e.setNeutral ();

  BOOST_CHECK (R6xSO3->nq () == 10);
  BOOST_CHECK (R6xSO3->nv () == 9);

  vector_t zero (9); zero.setZero ();
  BOOST_CHECK ((hpp::pinocchio::log (e) - zero).norm () < 1e-10);
}
