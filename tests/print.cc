// Copyright (c) 2017, Joseph Mirabel
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

#include <iostream>

#include <hpp/pinocchio/util.hh>

#include <hpp/pinocchio/fwd.hh>
#include <pinocchio/spatial/se3.hpp>

using namespace hpp;
using hpp::pinocchio::SE3;
using hpp::pinocchio::vector_t;
using hpp::pinocchio::matrix_t;
using hpp::pinocchio::vectorOut_t;
using hpp::pinocchio::vectorIn_t;

int main ()
{
//! [Example usage]
  std::cout << incindent
    << iendl << pretty_print (SE3::Identity())
    << iendl << condensed (SE3::Identity())
    << iendl << one_line (SE3::Identity())
    << decindent << iendl;

  std::cout << setpyformat
    << iendl << pretty_print (SE3::Identity())
    << iendl << condensed (SE3::Identity())
    << iendl << one_line (SE3::Identity())
    << iendl;

  vector_t v = vector_t::Ones(2);
  matrix_t m = matrix_t::Identity(3,3);

  std::cout << unsetpyformat
    << iendl << pretty_print (v)
    << iendl << condensed (v)
    << iendl << one_line (v.segment(0,2))
    << iendl << pretty_print (vectorOut_t(v))
    << iendl << condensed (vectorIn_t(v))
    << iendl;

  std::cout << setpyformat
    << iendl << pretty_print (m)
    << iendl << condensed (m)
    << iendl << one_line (m.block(1,1,2,2))
    << iendl;
//! [Example usage]

  return 0;
}
