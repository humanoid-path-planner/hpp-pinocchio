// Copyright (c) 2020, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#define BOOST_TEST_MODULE tserialization

#include <boost/test/unit_test.hpp>

#include <sstream>
#include <fstream>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <pinocchio/fwd.hpp>
#include <hpp/pinocchio/fwd.hh>

#include <hpp/pinocchio/serialization.hh>

using namespace hpp::pinocchio;

BOOST_AUTO_TEST_CASE(empty_array)
{
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

template<typename base_archive = boost::archive::xml_oarchive>
struct oarchive :
  base_archive, hpp::serialization::remove_duplicate::vector_archive
{
  oarchive(std::ostream& is) : base_archive (is) {}
};

template<typename Archive>
void check_remove_duplicate_impl ()
{
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

BOOST_AUTO_TEST_CASE(check_remove_duplicate)
{
  check_remove_duplicate_impl<boost::archive::xml_oarchive>();
  check_remove_duplicate_impl<oarchive<boost::archive::xml_oarchive> >();

  vector_t q (5);
  for (int i = 0; i < 10; ++i) {
    q.setRandom();
    vector_t qq;
    qq = q;

    hpp::serialization::remove_duplicate::eigen_compare<vector_t> cmp;
    BOOST_CHECK(!cmp(q, qq));
    BOOST_CHECK(!cmp(qq, q));
  }
}
