// Copyright (c) 2018, Joseph Mirabel
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

#define BOOST_TEST_MODULE tpool

#include <hpp/pinocchio/pool.hh>
#include <boost/test/unit_test.hpp>

template <typename T>
void compute (T& t, std::size_t i)
{
  t = 2*(T)i;
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
