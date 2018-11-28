// Copyright (c) 2018, Joseph Mirabel
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

#define BOOST_TEST_MODULE tpool

#include <hpp/pinocchio/pool.hh>
#include <boost/test/unit_test.hpp>

template <typename T>
void compute (T& t, std::size_t i)
{
  t = 2*i;
}

template <typename T>
void _test_pool ()
{
  using namespace hpp::pinocchio;
  typedef Pool<T> Pool_t;

  Pool_t pool;
  for (int i = 0; i < 4; ++i) pool.push_back (new T);

  const std::size_t N = 100;
  std::vector<T> res (N);

#pragma omp parallel for
  for (std::size_t i = 0; i < N; ++i) {
    T* t = pool.acquire ();
    compute(*t, i);
    res[i] = *t;
    pool.release (t);
  }

  T t;
  for (std::size_t i = 0; i < N; ++i) {
    compute (t, i);
    BOOST_CHECK_EQUAL (t, res[i]);
  }
}

BOOST_AUTO_TEST_CASE (test_pool)
{
  _test_pool<int>();
}
