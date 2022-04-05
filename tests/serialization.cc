// Copyright (c) 2020, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#define BOOST_TEST_MODULE tserialization

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/pinocchio/serialization.hh>
#include <pinocchio/fwd.hpp>
#include <sstream>

using namespace hpp::pinocchio;

BOOST_AUTO_TEST_CASE(empty_array) {
  Eigen::ArrayXi empty(0);

  std::stringstream ss;
  {
    boost::archive::xml_oarchive oa(ss);
    oa << boost::serialization::make_nvp("array", empty);
  }

  Eigen::ArrayXi empty_r;
  {
    boost::archive::xml_iarchive ia(ss);
    ia >> boost::serialization::make_nvp("array", empty_r);
  }

  BOOST_CHECK_EQUAL(empty_r.size(), 0);
}

template <typename base_archive = boost::archive::xml_oarchive>
struct oarchive : base_archive,
                  hpp::serialization::remove_duplicate::vector_archive {
  oarchive(std::ostream& is) : base_archive(is) {}
};

template <typename Archive>
void check_remove_duplicate_impl() {
  vector_t a(5), b(a);

  std::stringstream ss;
  {
    Archive oa(ss);
    hpp::serialization::remove_duplicate::serialize_vector(oa, "a", a, 0);
    hpp::serialization::remove_duplicate::serialize_vector(oa, "b", b, 0);
  }

  BOOST_TEST_MESSAGE(ss.str());

  vector_t a_r, b_r;
  {
    boost::archive::xml_iarchive ia(ss);
    hpp::serialization::remove_duplicate::serialize_vector(ia, "a", a_r, 0);
    hpp::serialization::remove_duplicate::serialize_vector(ia, "b", b_r, 0);
  }

  BOOST_CHECK_EQUAL(a, a_r);
  BOOST_CHECK_EQUAL(b, b_r);
}

BOOST_AUTO_TEST_CASE(check_remove_duplicate) {
  check_remove_duplicate_impl<boost::archive::xml_oarchive>();
  check_remove_duplicate_impl<oarchive<boost::archive::xml_oarchive> >();

  vector_t q(5);
  for (int i = 0; i < 10; ++i) {
    q.setRandom();
    vector_t qq;
    qq = q;

    hpp::serialization::remove_duplicate::eigen_compare<vector_t> cmp;
    BOOST_CHECK(!cmp(q, qq));
    BOOST_CHECK(!cmp(qq, q));
  }
}
