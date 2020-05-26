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
